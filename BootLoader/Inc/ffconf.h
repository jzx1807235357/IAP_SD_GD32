/*---------------------------------------------------------------------------/
/  FatFs Functional Configurations for BootLoader
/---------------------------------------------------------------------------*/

#define FFCONF_DEF	86604	/* Revision ID */

#define FF_INTDEF 1
#include "integer.h"

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_READONLY	0
/* Read-only configuration for BootLoader */

#define FF_FS_MINIMIZE	1
/* Minimize to reduce code size */

#define FF_USE_STRFUNC	0
/* Disable string functions */

#define FF_USE_FIND		0
/* Disable find functions */

#define FF_USE_MKFS		0
/* Disable f_mkfs() function */

#define FF_USE_EXPAND	0
/* Disable f_expand function */

#define FF_USE_CHMOD	0
/* Disable attribute manipulation */

#define FF_USE_LABEL	0
/* Disable volume label functions */

#define FF_USE_FORWARD	0
/* Disable f_forward() function */

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define FF_CODE_PAGE	437
/* U.S. code page - sufficient for firmware file names */

#define FF_USE_LFN		0
/* Disable LFN to reduce code size - use 8.3 format only */

#define FF_LFN_UNICODE	0
/* ANSI/OEM encoding */

#define FF_LFN_BUF		12
#define FF_SFN_BUF		12

#define FF_STRF_ENCODE	0

#define FF_FS_RPATH		0
/* Disable relative path */

/*---------------------------------------------------------------------------/
/ Drive/Volume Configurations
/---------------------------------------------------------------------------*/

#define FF_VOLUMES		1
/* Single volume (SD card only) */

#define FF_STR_VOLUME_ID	0

#define FF_MULTI_PARTITION	0

#define FF_MIN_SS		512
#define FF_MAX_SS		512
/* Fixed 512 byte sector size */

#define FF_USE_TRIM		0

#define FF_FS_NOFSINFO	0

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_TINY		1
/* Use tiny buffer configuration to reduce memory footprint */

#define FF_FS_EXFAT		0
/* Disable exFAT support */

#define FF_FS_NORTC		1
/* No RTC in BootLoader */

#define FF_NORTC_MON	1
#define FF_NORTC_MDAY	1
#define FF_NORTC_YEAR	2025

#define FF_FS_LOCK		0
/* No file locking needed */

#define FF_FS_REENTRANT	0
/* Non-reentrant mode - no RTOS in BootLoader */

#define FF_FS_TIMEOUT	0

/*--- End of configuration options ---*/
