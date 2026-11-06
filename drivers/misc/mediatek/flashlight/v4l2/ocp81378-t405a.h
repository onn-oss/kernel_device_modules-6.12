#ifndef _OCP81378_T405A_H
#define _OCP81378_T405A_H

#include "flashlight-t405a.h"

/*  FLASH Brightness for OCP81378
 *	min 15000uA, step 15650uA, max 2000000uA
 */
#define OCP81378_FLASH_BRT_MIN                  15000
#define OCP81378_FLASH_BRT_STEP                 15650
#define OCP81378_FLASH_BRT_MAX                  2000000
#define OCP81378_FLASH_BRT_uA_TO_REG(a)	\
	((a) < OCP81378_FLASH_BRT_MIN ? 0 :	\
	 (((a) - OCP81378_FLASH_BRT_MIN) / OCP81378_FLASH_BRT_STEP))
#define OCP81378_FLASH_BRT_REG_TO_uA(a)		\
	((a) * OCP81378_FLASH_BRT_STEP + OCP81378_FLASH_BRT_MIN)

/*  TORCH BRT for OCP81378
 *	min 2900uA, step 3840uA, max 490000uA
 */
#define OCP81378_TORCH_BRT_MIN                  2900
#define OCP81378_TORCH_BRT_STEP                 3840
#define OCP81378_TORCH_BRT_MAX                  490000
#define OCP81378_TORCH_BRT_uA_TO_REG(a)	\
	((a) < OCP81378_TORCH_BRT_MIN ? 0 :	\
	 (((a) - OCP81378_TORCH_BRT_MIN) / OCP81378_TORCH_BRT_STEP))
#define OCP81378_TORCH_BRT_REG_TO_uA(a)		\
	((a) * OCP81378_TORCH_BRT_STEP + OCP81378_TORCH_BRT_MIN)

#define OCP81378_BRT_REG_MASK                   0x7F
/* Torch mode default current 100mA */
#define OCP81378_TORCH_MODE_DEFAULT_CURRENT     OCP81378_TORCH_BRT_uA_TO_REG(100000)

#endif /* _OCP81378_T405A_H */
