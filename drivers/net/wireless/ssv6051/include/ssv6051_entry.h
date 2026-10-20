/* SPDX-License-Identifier: BSD-2-Clause OR GPL-2.0-only */
/*
 * SSV6051 cross-module entry points.
 *
 * These functions are used by ssv6051-generic-wlan.c, ssvdevice/ssvdevice.c
 * and hwif/sdio/sdio.c to wire the driver up, but were previously declared
 * via scattered extern statements in .c files. Group them here.
 */
#ifndef __SSV6051_ENTRY_H__
#define __SSV6051_ENTRY_H__

/* implemented in ssvdevice/ssvdevice.c */
int ssvdevice_init(void);
void ssvdevice_exit(void);

/* implemented in hwif/sdio/sdio.c */
int ssv6xxx_sdio_init(void);
void ssv6xxx_sdio_exit(void);
int ssv6xxx_get_dev_status(void);

/* implemented in hci/ssv_hci.c */
int ssv6xxx_hci_init(void);
void ssv6xxx_hci_exit(void);

/* implemented in smac/init.c */
int ssv6xxx_init(void);
void ssv6xxx_exit(void);

#endif /* __SSV6051_ENTRY_H__ */
