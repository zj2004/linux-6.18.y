// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/net/phy/maxio.c
 *
 * Driver for Maxio MAE0621A Gigabit Ethernet PHYs (Q2C / Q3C / Q3I).
 *
 * Datasheet: MAE0621A-Q3C/Q3I Gigabit Ethernet Transceiver Datasheet v1.1
 *
 * The original vendor driver used open coded page switching, printk()
 * logging and a private (and incomplete) MMD accessor.  This version
 * follows the mainline style of the rtl8211f driver: the phylib paged
 * helpers are used throughout, every register access is checked, and the
 * EEE / LED handling is expressed through the generic phylib APIs.
 *
 * The file is intentionally written so that it builds both against the
 * 6.12 vendor kernel and against 6.18, where the EEE advertisement
 * handling was reworked (see mae0621a_disable_eee()).
 *
 * Author: unifreq <flippy@sina.com>
 *
 * Copyright (c) 2021 Maxio Technology, Inc.
 * Copyright (c) 2026 unifreq <flippy@sina.com>
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/version.h>

#define MAXIO_PHY_VER				"v2.0.0"

/* Page select register, present on every page (PAGSR). */
#define MAE0621A_PAGE_SELECT			0x1f

/* PHY identifiers.  Bit 31 carries the silicon revision, mask it out. */
#define MAE0621A_PHY_ID_Q2C			0x7b744411
#define MAE0621A_PHY_ID_Q3C			0x7b744412
#define MAE0621A_PHY_ID_MASK			0x7fffffff

/*
 * PHY specific registers (datasheet chapter 7).
 *
 * PHYCR1/PHYCR2 live on page 0xa43, the interrupt registers are reachable
 * from the default page 0xa42, the LED control register is on page 0xd04.
 */
#define MAE0621A_PHYCR1				0x18
#define MAE0621A_PHYCR2				0x19
#define MAE0621A_PHYCR1_ALDPS_EN		BIT(2)
#define MAE0621A_PHYCR1_ALDPS_PLL_OFF		BIT(1)
#define MAE0621A_PHYCR2_CLKOUT_125M		BIT(11)
#define MAE0621A_PHYCR2_CLKOUT_EN		BIT(0)

/* CLKOUT enabled, 125MHz, 25MHz reference: value used by the vendor code. */
#define MAE0621A_CLKOUT_CFG			0x0823

#define MAE0621A_INER				0x12
#define MAE0621A_INSR				0x1d

#define MAE0621A_LED_COUNT			3

/*
 * LED Control Register (LCR), page 0xd04, register 0x10.
 *
 *	15    LED_MODE    0 = mode A, 1 = mode B
 *	14    LED2_ACT    (mode B) / LED2_LINK (mode A)
 *	13    LED2_LINK1000
 *	11    LED2_LINK100
 *	10    LED2_LINK10
 *	 9    LED1_ACT
 *	 8    LED1_LINK1000
 *	 6    LED1_LINK100
 *	 5    LED1_LINK10
 *	 4    LED0_ACT
 *	 3    LED0_LINK1000
 *	 1    LED0_LINK100
 *	 0    LED0_LINK10
 *
 * Each LED occupies five bits (bit 2 / bit 7 / bit 12 are reserved), so the
 * layout is bit for bit identical to the RTL8211F LEDCR and the very same
 * trigger mapping can be reused.
 */
#define MAE0621A_LEDCR				0x10
#define MAE0621A_LEDCR_MODE			BIT(15)
#define MAE0621A_LEDCR_ACT_TXRX			BIT(4)
#define MAE0621A_LEDCR_LINK_1000		BIT(3)
#define MAE0621A_LEDCR_LINK_100			BIT(1)
#define MAE0621A_LEDCR_LINK_10			BIT(0)
#define MAE0621A_LEDCR_MASK			GENMASK(4, 0)
#define MAE0621A_LEDCR_SHIFT			5
#define MAE0621A_LEDCR_PAGE			0xd04

struct maxio_priv {
	/* "maxio,led-data" style: raw 16 bit LCR value written verbatim */
	bool led_data_cfg;
	u16 led_data;
	/* "maxio,ledX-triggers" style: per-LED trigger configuration */
	bool led_dts_cfg[MAE0621A_LED_COUNT];
	u16 led_vals[MAE0621A_LED_COUNT];
};

/*
 * LED support.  Two DT styles are accepted at the same time:
 *
 *	- raw style:     "maxio,led-data" holds the complete 16 bit LCR value
 *	- trigger style: "maxio,led0/1/2-triggers" string lists
 *
 * The trigger style wins over the raw style for the LEDs it configures.
 * Both are re-applied after any reset from mae0621a_restore_leds().
 */
static const struct {
	const char *name;
	u16 mask;
} mae0621a_led_trigger_map[] = {
	{ "activity", MAE0621A_LEDCR_ACT_TXRX },
	{ "rx", MAE0621A_LEDCR_ACT_TXRX },
	{ "tx", MAE0621A_LEDCR_ACT_TXRX },
	{ "link", MAE0621A_LEDCR_LINK_10 | MAE0621A_LEDCR_LINK_100 |
		  MAE0621A_LEDCR_LINK_1000 },
	{ "link-10", MAE0621A_LEDCR_LINK_10 },
	{ "link-100", MAE0621A_LEDCR_LINK_100 },
	{ "link-1000", MAE0621A_LEDCR_LINK_1000 },
};

static const char * const mae0621a_led_propnames[MAE0621A_LED_COUNT] = {
	"maxio,led0-triggers",
	"maxio,led1-triggers",
	"maxio,led2-triggers",
};

/* ------------------------------------------------------------------ */
/* Register access helpers                                            */
/* ------------------------------------------------------------------ */

static int maxio_read_page(struct phy_device *phydev)
{
	return __phy_read(phydev, MAE0621A_PAGE_SELECT);
}

static int maxio_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, MAE0621A_PAGE_SELECT, page);
}

static int maxio_read_paged(struct phy_device *phydev, int page, u32 regnum)
{
	int oldpage, ret;

	oldpage = phy_select_page(phydev, page);
	if (oldpage < 0)
		return oldpage;

	ret = __phy_read(phydev, regnum);

	return phy_restore_page(phydev, oldpage, ret);
}

static int maxio_write_paged(struct phy_device *phydev, int page,
			     u32 regnum, u16 val)
{
	int oldpage, ret;

	oldpage = phy_select_page(phydev, page);
	if (oldpage < 0)
		return oldpage;

	ret = __phy_write(phydev, regnum, val);

	return phy_restore_page(phydev, oldpage, ret);
}

/*
 * MMD access.  The callbacks run with phydev->lock already held by the
 * phylib, therefore only __phy_read()/__phy_write() may be used here.
 */
static int maxio_read_mmd(struct phy_device *phydev, int devnum, u16 regnum)
{
	int ret;

	ret = __phy_write(phydev, MII_MMD_CTRL, devnum);
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, MII_MMD_DATA, regnum);
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, MII_MMD_CTRL,
			  MII_MMD_CTRL_NOINCR | devnum);
	if (ret < 0)
		return ret;

	return __phy_read(phydev, MII_MMD_DATA);
}

static int maxio_write_mmd(struct phy_device *phydev, int devnum, u16 regnum,
			   u16 val)
{
	int ret;

	ret = __phy_write(phydev, MII_MMD_CTRL, devnum);
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, MII_MMD_DATA, regnum);
	if (ret < 0)
		return ret;

	ret = __phy_write(phydev, MII_MMD_CTRL,
			  MII_MMD_CTRL_NOINCR | devnum);
	if (ret < 0)
		return ret;

	return __phy_write(phydev, MII_MMD_DATA, val);
}

/* ------------------------------------------------------------------ */
/* EEE                                                                */
/* ------------------------------------------------------------------ */

/*
 * The MAE0621A requires EEE to be switched off entirely.
 *
 * The phylib EEE advertisement API changed over time:
 *
 *  - up to and including 6.14 the phy_device carried a raw u32 bitmap of
 *    "broken" EEE modes (phydev->eee_broken_modes, holding MDIO_AN_EEE_ADV
 *    bit values).  genphy_config_eee_advert() / genphy_c45_write_eee_adv()
 *    mask those bits out of the advertisement.  Note that in the 6.12
 *    rockchip tree genphy_config_aneg() is a static inline that only
 *    reaches the write through genphy_c45_an_config_eee_aneg(), which
 *    requires phydev->supported_eee to be populated - something a plain
 *    C22 PHY such as the MAE0621A never reports.
 *
 *  - since 6.15 the bitmap was replaced by the linkmode based
 *    phydev->eee_disabled_modes and the phy_disable_eee_mode() helper.
 *
 * Because the C22 MAE0621A does not advertise EEE capabilities through the
 * C45 registers, neither path is guaranteed to program the PHY.  Therefore
 * the advertisement register is additionally cleared explicitly, exactly
 * like the original vendor driver did, so the PHY really stops advertising
 * 100BASE-TX / 1000BASE-T EEE.
 */
static void mae0621a_disable_eee(struct phy_device *phydev)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 15, 0)
	phy_disable_eee_mode(phydev, ETHTOOL_LINK_MODE_100baseT_Full_BIT);
	phy_disable_eee_mode(phydev, ETHTOOL_LINK_MODE_1000baseT_Full_BIT);
#else
	phydev->eee_broken_modes |= MDIO_EEE_100TX | MDIO_EEE_1000T;
#endif

	/* Clear the PHY side advertisement (MMD 3, register 0x3c). */
	phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV, 0);
}

/* ------------------------------------------------------------------ */
/* LED helpers                                                        */
/* ------------------------------------------------------------------ */

static int mae0621a_parse_led_data(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct maxio_priv *priv = phydev->priv;
	u32 led_data;
	int ret;

	if (!priv)
		return 0;

	ret = device_property_read_u32(dev, "maxio,led-data", &led_data);
	if (ret)
		return 0;

	ret = phy_write_paged(phydev, MAE0621A_LEDCR_PAGE, MAE0621A_LEDCR,
			      (u16)led_data);
	if (ret < 0) {
		phydev_err(phydev, "Failed to write LEDCR (led-data), err=%d\n",
			   ret);
		return ret;
	}

	priv->led_data = (u16)led_data;
	priv->led_data_cfg = true;
	phydev_dbg(phydev, "LEDCR configured via DTS (led-data), val=0x%04x\n",
		   priv->led_data);

	return 0;
}

static int mae0621a_parse_led_triggers(struct phy_device *phydev, u8 led_index,
				       const char *propname)
{
	struct device *dev = &phydev->mdio.dev;
	struct maxio_priv *priv = phydev->priv;
	const char **trigger_names;
	int count, i, j;
	u16 mask, val = 0;
	int ret;

	if (!priv)
		return 0;

	count = device_property_string_array_count(dev, propname);
	if (count <= 0)
		return 0;

	trigger_names = kcalloc(count, sizeof(char *), GFP_KERNEL);
	if (!trigger_names)
		return -ENOMEM;

	ret = device_property_read_string_array(dev, propname, trigger_names,
						count);
	if (ret < 0) {
		kfree(trigger_names);
		return ret;
	}

	for (i = 0; i < count; i++) {
		bool found = false;

		for (j = 0; j < ARRAY_SIZE(mae0621a_led_trigger_map); j++) {
			if (strcmp(trigger_names[i],
				   mae0621a_led_trigger_map[j].name) == 0) {
				val |= mae0621a_led_trigger_map[j].mask;
				found = true;
				break;
			}
		}
		if (!found)
			phydev_warn(phydev, "Unknown LED trigger '%s' for LED%d\n",
				    trigger_names[i], led_index);
	}
	kfree(trigger_names);

	val &= MAE0621A_LEDCR_MASK;
	mask = MAE0621A_LEDCR_MASK << (MAE0621A_LEDCR_SHIFT * led_index);

	ret = phy_modify_paged(phydev, MAE0621A_LEDCR_PAGE, MAE0621A_LEDCR,
			       mask, (val << (MAE0621A_LEDCR_SHIFT * led_index)) |
			       MAE0621A_LEDCR_MODE);
	if (ret < 0) {
		phydev_err(phydev, "Failed to write LED%d config, err=%d\n",
			   led_index, ret);
		return ret;
	}

	priv->led_vals[led_index] = val;
	priv->led_dts_cfg[led_index] = true;
	phydev_dbg(phydev, "LED%d configured via DTS with triggers, val=0x%04x\n",
		   led_index, val);

	return 0;
}

static int mae0621a_parse_leds(struct phy_device *phydev)
{
	int i, ret;

	ret = mae0621a_parse_led_data(phydev);
	if (ret < 0)
		return ret;

	for (i = 0; i < MAE0621A_LED_COUNT; i++)
		mae0621a_parse_led_triggers(phydev, i,
					    mae0621a_led_propnames[i]);

	return 0;
}

static void mae0621a_restore_leds(struct phy_device *phydev)
{
	struct maxio_priv *priv = phydev->priv;
	int i, ret;

	if (!priv)
		return;

	if (priv->led_data_cfg) {
		ret = phy_write_paged(phydev, MAE0621A_LEDCR_PAGE,
				      MAE0621A_LEDCR, priv->led_data);
		if (ret < 0)
			phydev_err(phydev, "Failed to restore LEDCR (led-data): %d\n",
				   ret);
		return;
	}

	for (i = 0; i < MAE0621A_LED_COUNT; i++) {
		u16 mask, val;

		if (!priv->led_dts_cfg[i])
			continue;

		mask = MAE0621A_LEDCR_MASK << (MAE0621A_LEDCR_SHIFT * i);
		val = (priv->led_vals[i] & MAE0621A_LEDCR_MASK) <<
		      (MAE0621A_LEDCR_SHIFT * i);
		val |= MAE0621A_LEDCR_MODE;

		ret = phy_modify_paged(phydev, MAE0621A_LEDCR_PAGE,
				       MAE0621A_LEDCR, mask, val);
		if (ret < 0)
			phydev_err(phydev, "Failed to restore LED%d config: %d\n",
				   i, ret);
	}
}

static bool mae0621a_led_dts_managed(struct maxio_priv *priv, u8 index)
{
	if (!priv)
		return false;

	if (priv->led_data_cfg)
		return true;

	return index < MAE0621A_LED_COUNT && priv->led_dts_cfg[index];
}

static int mae0621a_led_hw_is_supported(struct phy_device *phydev, u8 index,
					unsigned long rules)
{
	struct maxio_priv *priv = phydev->priv;
	const unsigned long mask = BIT(TRIGGER_NETDEV_LINK_10) |
				   BIT(TRIGGER_NETDEV_LINK_100) |
				   BIT(TRIGGER_NETDEV_LINK_1000) |
				   BIT(TRIGGER_NETDEV_RX) |
				   BIT(TRIGGER_NETDEV_TX);

	/*
	 * The MAE0621A supports up to three LEDs with the same semantics as
	 * the RTL8211F:
	 *  - Link:   configurable subset of 10/100/1000 link rates
	 *  - Active: blink on activity, RX and TX are not distinguished
	 * Only mode B is used here.
	 */
	if (mae0621a_led_dts_managed(priv, index))
		return -EOPNOTSUPP;

	if (index >= MAE0621A_LED_COUNT)
		return -EINVAL;

	/* Filter out any other unsupported triggers. */
	if (rules & ~mask)
		return -EOPNOTSUPP;

	/* RX and TX are not differentiated, either both are set or not set. */
	if (!(rules & BIT(TRIGGER_NETDEV_RX)) ^
	    !(rules & BIT(TRIGGER_NETDEV_TX)))
		return -EOPNOTSUPP;

	return 0;
}

static int mae0621a_led_hw_control_get(struct phy_device *phydev, u8 index,
				       unsigned long *rules)
{
	struct maxio_priv *priv = phydev->priv;
	int val;

	if (mae0621a_led_dts_managed(priv, index))
		return -EOPNOTSUPP;

	if (index >= MAE0621A_LED_COUNT)
		return -EINVAL;

	val = phy_read_paged(phydev, MAE0621A_LEDCR_PAGE, MAE0621A_LEDCR);
	if (val < 0)
		return val;

	val >>= MAE0621A_LEDCR_SHIFT * index;
	val &= MAE0621A_LEDCR_MASK;

	if (val & MAE0621A_LEDCR_LINK_10)
		set_bit(TRIGGER_NETDEV_LINK_10, rules);

	if (val & MAE0621A_LEDCR_LINK_100)
		set_bit(TRIGGER_NETDEV_LINK_100, rules);

	if (val & MAE0621A_LEDCR_LINK_1000)
		set_bit(TRIGGER_NETDEV_LINK_1000, rules);

	if (val & MAE0621A_LEDCR_ACT_TXRX) {
		set_bit(TRIGGER_NETDEV_RX, rules);
		set_bit(TRIGGER_NETDEV_TX, rules);
	}

	return 0;
}

static int mae0621a_led_hw_control_set(struct phy_device *phydev, u8 index,
				       unsigned long rules)
{
	const u16 mask = MAE0621A_LEDCR_MASK << (MAE0621A_LEDCR_SHIFT * index);
	struct maxio_priv *priv = phydev->priv;
	u16 reg = 0;

	if (mae0621a_led_dts_managed(priv, index))
		return -EOPNOTSUPP;

	if (index >= MAE0621A_LED_COUNT)
		return -EINVAL;

	if (test_bit(TRIGGER_NETDEV_LINK_10, &rules))
		reg |= MAE0621A_LEDCR_LINK_10;

	if (test_bit(TRIGGER_NETDEV_LINK_100, &rules))
		reg |= MAE0621A_LEDCR_LINK_100;

	if (test_bit(TRIGGER_NETDEV_LINK_1000, &rules))
		reg |= MAE0621A_LEDCR_LINK_1000;

	if (test_bit(TRIGGER_NETDEV_RX, &rules) ||
	    test_bit(TRIGGER_NETDEV_TX, &rules))
		reg |= MAE0621A_LEDCR_ACT_TXRX;

	reg <<= MAE0621A_LEDCR_SHIFT * index;
	reg |= MAE0621A_LEDCR_MODE;	 /* Mode B */

	return phy_modify_paged(phydev, MAE0621A_LEDCR_PAGE, MAE0621A_LEDCR,
				mask, reg);
}

/* ------------------------------------------------------------------ */
/* ADC self check (Q2C only)                                          */
/* ------------------------------------------------------------------ */

static int mae0621a_adc_check(struct phy_device *phydev)
{
	static const u16 test_vals[] = { 0xf908, 0xfa08, 0xfb08, 0xfc08 };
	int i, ret;

	ret = maxio_write_paged(phydev, 0xd96, 0x02, 0x1fff);
	if (ret < 0)
		return ret;

	ret = maxio_write_paged(phydev, 0xd96, 0x02, 0x1000);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(test_vals); i++) {
		ret = maxio_write_paged(phydev, 0xd8f, 0x0b, test_vals[i]);
		if (ret < 0)
			return ret;

		ret = maxio_read_paged(phydev, 0xd92, 0x0b);
		if (ret < 0)
			return ret;

		if (!(ret & 0x01ff))
			return -EAGAIN;
	}

	return 0;
}

static int mae0621a_self_check(struct phy_device *phydev)
{
	int i, ret = -EAGAIN;

	for (i = 0; i < 50; i++) {
		ret = mae0621a_adc_check(phydev);
		if (!ret)
			break;

		ret = maxio_write_paged(phydev, 0x0, MII_BMCR, 0x1940);
		if (ret < 0)
			return ret;

		msleep(10);

		ret = maxio_write_paged(phydev, 0x0, MII_BMCR, 0x1140);
		if (ret < 0)
			return ret;

		ret = maxio_write_paged(phydev, 0x0, MII_BMCR, 0x9140);
		if (ret < 0)
			return ret;
	}

	ret = maxio_write_paged(phydev, 0xd96, 0x02, 0x0fff);
	if (ret < 0)
		return ret;

	ret = maxio_write_paged(phydev, 0x0, MII_BMCR, 0x9140);
	if (ret < 0)
		return ret;

	return maxio_write_page(phydev, 0);
}

/* ------------------------------------------------------------------ */
/* Per variant initialisation                                         */
/* ------------------------------------------------------------------ */

static int mae0621a_q2c_config_init(struct phy_device *phydev)
{
	int ret;

	phydev_info(phydev, "initializing Maxio MAE0621A-Q2C PHY (%s)\n",
		    MAXIO_PHY_VER);

	/* MDC/auto-speed-down, crystal and analogue front-end tuning */
	ret = maxio_write_paged(phydev, 0xda0, 0x10, 0x0f13);
	if (ret < 0)
		return ret;

	ret = maxio_write_paged(phydev, 0xd92, 0x02, 0x200a);
	if (ret < 0)
		return ret;

	mae0621a_disable_eee(phydev);

	ret = maxio_write_paged(phydev, 0xd8f, 0x00, 0x0300);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd90, 0x02, 0x1555);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd90, 0x05, 0x2b15);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd96, 0x13, 0x07bc);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd8f, 0x08, 0x2500);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd91, 0x06, 0x6880);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd92, 0x14, 0x000a);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd91, 0x07, 0x5b00);
	if (ret < 0)
		return ret;

	/* CLKOUT enabled, 125MHz */
	ret = maxio_write_paged(phydev, 0xa43, MAE0621A_PHYCR2,
				MAE0621A_CLKOUT_CFG);
	if (ret < 0)
		return ret;

	ret = phy_write(phydev, MAE0621A_PAGE_SELECT, 0);
	if (ret < 0)
		return ret;

	ret = mae0621a_self_check(phydev);
	if (ret < 0)
		phydev_warn(phydev, "ADC self check did not report ready: %d\n",
			    ret);

	/* Re-apply the LED configuration after the resets above. */
	mae0621a_restore_leds(phydev);

	msleep(100);

	return 0;
}

static int mae0621a_q3c_config_init(struct phy_device *phydev)
{
	int ret;

	phydev_info(phydev, "initializing Maxio MAE0621A-Q3C/Q3I PHY (%s)\n",
		    MAXIO_PHY_VER);

	/* MDC and auto-speed-down */
	ret = maxio_write_paged(phydev, 0xdab, 0x17, 0x0f13);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd8f, 0x10, 0x0300);
	if (ret < 0)
		return ret;

	/* analogue front-end tuning */
	ret = maxio_write_paged(phydev, 0xd96, 0x15, 0xc08a);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xda4, 0x12, 0x07bc);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd8f, 0x16, 0x2500);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd90, 0x16, 0x1555);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd92, 0x11, 0x2b15);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd96, 0x16, 0x4010);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xda5, 0x11, 0x4a12);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xda5, 0x12, 0x4a12);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xda8, 0x11, 0x0175);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd99, 0x16, 0x000a);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xd95, 0x13, 0x5b00);
	if (ret < 0)
		return ret;

	/* CLKOUT enabled, 125MHz */
	ret = maxio_write_paged(phydev, 0xa43, MAE0621A_PHYCR2,
				MAE0621A_CLKOUT_CFG);
	if (ret < 0)
		return ret;

	/* soft reset / restart auto-negotiation */
	ret = maxio_write_paged(phydev, 0x0, MII_BMCR, 0x9140);
	if (ret < 0)
		return ret;

	ret = phy_write(phydev, MAE0621A_PAGE_SELECT, 0);
	if (ret < 0)
		return ret;

	/* Re-apply the LED configuration after the reset above. */
	mae0621a_restore_leds(phydev);

	msleep(100);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Power management                                                   */
/* ------------------------------------------------------------------ */

static int mae0621a_q2c_suspend(struct phy_device *phydev)
{
	int ret;

	ret = genphy_suspend(phydev);
	if (ret < 0)
		return ret;

	/* Back to page 0. */
	return phy_write(phydev, MAE0621A_PAGE_SELECT, 0);
}

static int mae0621a_q2c_resume(struct phy_device *phydev)
{
	int ret;

	ret = genphy_resume(phydev);
	if (ret < 0)
		return ret;

	/* Soft reset to reload the internal analogue settings. */
	ret = phy_set_bits(phydev, MII_BMCR, BMCR_RESET);
	if (ret < 0)
		return ret;

	msleep(20);
	mae0621a_restore_leds(phydev);

	return 0;
}

static int mae0621a_q3c_suspend(struct phy_device *phydev)
{
	int ret;

	ret = maxio_write_paged(phydev, 0xdaa, 0x17, 0x1011);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xdab, 0x15, 0x5550);
	if (ret < 0)
		return ret;

	ret = phy_write(phydev, MAE0621A_PAGE_SELECT, 0);
	if (ret < 0)
		return ret;

	ret = genphy_suspend(phydev);
	if (ret < 0)
		return ret;

	return phy_write(phydev, MAE0621A_PAGE_SELECT, 0);
}

static int mae0621a_q3c_resume(struct phy_device *phydev)
{
	int ret;

	ret = genphy_resume(phydev);
	if (ret < 0)
		return ret;

	ret = maxio_write_paged(phydev, 0xdaa, 0x17, 0x1001);
	if (ret < 0)
		return ret;
	ret = maxio_write_paged(phydev, 0xdab, 0x15, 0x0000);
	if (ret < 0)
		return ret;

	mae0621a_restore_leds(phydev);

	return phy_write(phydev, MAE0621A_PAGE_SELECT, 0);
}

/* ------------------------------------------------------------------ */
/* Probe                                                              */
/* ------------------------------------------------------------------ */

static int mae0621a_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct maxio_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;

	return mae0621a_parse_leds(phydev);
}

/* ------------------------------------------------------------------ */
/* Driver registration                                                */
/* ------------------------------------------------------------------ */

static struct phy_driver maxio_drvs[] = {
	{
		.phy_id		= MAE0621A_PHY_ID_Q2C,
		.phy_id_mask	= MAE0621A_PHY_ID_MASK,
		.name		= "Maxio MAE0621A-Q2C Gigabit Ethernet",
		.features	= PHY_GBIT_FEATURES,
		.probe		= mae0621a_probe,
		.config_init	= mae0621a_q2c_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.suspend	= mae0621a_q2c_suspend,
		.resume		= mae0621a_q2c_resume,
		.read_page	= maxio_read_page,
		.write_page	= maxio_write_page,
		.read_mmd	= maxio_read_mmd,
		.write_mmd	= maxio_write_mmd,
		.flags		= PHY_ALWAYS_CALL_SUSPEND,
		.led_hw_is_supported = mae0621a_led_hw_is_supported,
		.led_hw_control_get = mae0621a_led_hw_control_get,
		.led_hw_control_set = mae0621a_led_hw_control_set,
	},
	{
		.phy_id		= MAE0621A_PHY_ID_Q3C,
		.phy_id_mask	= MAE0621A_PHY_ID_MASK,
		.name		= "Maxio MAE0621A/B-Q3C(I) Gigabit Ethernet",
		.features	= PHY_GBIT_FEATURES,
		.probe		= mae0621a_probe,
		.config_init	= mae0621a_q3c_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.suspend	= mae0621a_q3c_suspend,
		.resume		= mae0621a_q3c_resume,
		.read_page	= maxio_read_page,
		.write_page	= maxio_write_page,
		.read_mmd	= maxio_read_mmd,
		.write_mmd	= maxio_write_mmd,
		.flags		= PHY_ALWAYS_CALL_SUSPEND,
		.led_hw_is_supported = mae0621a_led_hw_is_supported,
		.led_hw_control_get = mae0621a_led_hw_control_get,
		.led_hw_control_set = mae0621a_led_hw_control_set,
	},
};
module_phy_driver(maxio_drvs);

static struct mdio_device_id __maybe_unused maxio_tbl[] = {
	{ MAE0621A_PHY_ID_Q2C, MAE0621A_PHY_ID_MASK },
	{ MAE0621A_PHY_ID_Q3C, MAE0621A_PHY_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, maxio_tbl);

MODULE_DESCRIPTION("Maxio MAE0621A PHY driver");
MODULE_AUTHOR("unifreq <flippy@sina.com>");
MODULE_LICENSE("GPL");
