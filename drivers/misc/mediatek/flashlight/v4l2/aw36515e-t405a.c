#include "aw36515e-t405a.h"

const int aw36515e_current[FLASHLIGHT_LEVEL_NUM] = {
	234, 250, 265, 297, 328, 359, 406, 500
};

const unsigned char aw36515e_flash_level[FLASHLIGHT_LEVEL_NUM] = {
	0x1D, 0x1F, 0x21, 0x25, 0x29, 0x2D, 0x33, 0x3F
};

unsigned char aw36515e_flash_brt_uA_2_reg(unsigned int flash_current)
{
    unsigned int cur = 0;

    if(flash_current > AW36515E_FLASH_BRT_MAX / 1000)
        cur = AW36515E_FLASH_BRT_MAX / 1000;

    return AW36515E_FLASH_BRT_uA_TO_REG(cur);
}

unsigned char aw36515e_torch_brt_uA_2_reg(unsigned int torch_current)
{
    unsigned int cur = 0;

    if(torch_current > AW36515E_TORCH_BRT_MAX / 1000)
        cur = AW36515E_TORCH_BRT_MAX / 1000;

    return AW36515E_TORCH_BRT_uA_TO_REG(cur);
}
