#ifndef __MT65XX_LCM_LIST_H__
#define __MT65XX_LCM_LIST_H__

#include <lcm_drv.h>

#if defined(MTK_LCM_DEVICE_TREE_SUPPORT)
extern LCM_DRIVER lcm_common_drv;
#else
extern LCM_DRIVER boyi_otm1287_hd720_dsi_vdo_lcm_drv;
extern LCM_DRIVER lide_rm68200_hd720_dsi_vdo_lcm_drv;
extern LCM_DRIVER boyi_otm1901_fhd_dsi_vdo_lcm_drv;
extern LCM_DRIVER bld_nt35532_fhd_dsi_vdo_lcm_drv;
extern LCM_DRIVER lide_ili9885_fhd_dsi_vdo_lcm_drv;
#endif

#ifdef BUILD_LK
extern void mdelay(unsigned long msec);
#endif

#endif
