/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Allwinner sunxi SID (Security ID) support
 */

#ifndef _LINUX_SUNXI_SID_H
#define _LINUX_SUNXI_SID_H

int sunxi_get_soc_chipid(unsigned char *chipid);
int sunxi_get_serial(unsigned char *serial);

#endif /* _LINUX_SUNXI_SID_H */
