#include "ocp81378-t405a.h"

const int ocp81378_current[FLASHLIGHT_LEVEL_NUM] = {
	234, 250, 265, 297, 328, 359, 406, 500
};

const unsigned char ocp81378_flash_level[FLASHLIGHT_LEVEL_NUM] = {
	0x0E, 0x0F, 0x10, 0x12, 0x14, 0x16, 0x19, 0x1F
};

unsigned char ocp81378_flash_brt_uA_2_reg(unsigned int flash_current)
{
    unsigned int cur = 0;

    if(flash_current > OCP81378_FLASH_BRT_MAX / 1000)
        cur = OCP81378_FLASH_BRT_MAX / 1000;

    return OCP81378_FLASH_BRT_uA_TO_REG(cur);
}

unsigned char ocp81378_torch_brt_uA_2_reg(unsigned int torch_current)
{
    unsigned int cur = 0;

    if(torch_current > OCP81378_TORCH_BRT_MAX / 1000)
        cur = OCP81378_TORCH_BRT_MAX / 1000;

    return OCP81378_TORCH_BRT_uA_TO_REG(cur);
}
