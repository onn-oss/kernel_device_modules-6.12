#ifndef _AW36515E_T405A_H
#define _AW36515E_T405A_H

#include "flashlight-t405a.h"

/*  FLASH Brightness for AW36515E
 *	min 15000uA, step 15650uA, max 2000000uA
 */
#define AW36515E_FLASH_BRT_MIN                  3910
#define AW36515E_FLASH_BRT_STEP                 7830
#define AW36515E_FLASH_BRT_MAX                  2000000
#define AW36515E_FLASH_BRT_uA_TO_REG(a)	\
	((a) < AW36515E_FLASH_BRT_MIN ? 0 :	\
	 (((a) - AW36515E_FLASH_BRT_MIN) / AW36515E_FLASH_BRT_STEP))
#define AW36515E_FLASH_BRT_REG_TO_uA(a)		\
	((a) * AW36515E_FLASH_BRT_STEP + AW36515E_FLASH_BRT_MIN)

/*  TORCH BRT for AW36515E
 *	min 2900uA, step 3840uA, max 490000uA
 */
#define AW36515E_TORCH_BRT_MIN                  980
#define AW36515E_TORCH_BRT_STEP                 1960
#define AW36515E_TORCH_BRT_MAX                  500000
#define AW36515E_TORCH_BRT_uA_TO_REG(a)	\
	((a) < AW36515E_TORCH_BRT_MIN ? 0 :	\
	 (((a) - AW36515E_TORCH_BRT_MIN) / AW36515E_TORCH_BRT_STEP))
#define AW36515E_TORCH_BRT_REG_TO_uA(a)		\
	((a) * AW36515E_TORCH_BRT_STEP + AW36515E_TORCH_BRT_MIN)

#define AW36515E_BRT_REG_MASK                   0xFF
/* Torch mode default current 100mA */
#define AW36515E_TORCH_MODE_DEFAULT_CURRENT     AW36515E_TORCH_BRT_uA_TO_REG(100000)

#endif /* _AW36515E_T405A_H */
