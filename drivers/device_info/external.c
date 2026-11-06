#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <asm/setup.h>

#include "dev_info.h"
#include "external.h"
/*******************************************************************************************/

// liujia, UGFDDS-1603, 2023-7-31, sta
#define ATTR_SUSBDL_DISABLE                 0x00
#define ATTR_SUSBDL_ENABLE                  0x11
#define ATTR_SUSBDL_ONLY_ENABLE_ON_SCHIP    0x22

#define ATTR_SBOOT_DISABLE                  0x00
#define ATTR_SBOOT_ENABLE                   0x11
#define ATTR_SBOOT_ONLY_ENABLE_ON_SCHIP     0x22

#define NUM_SBC_PUBK_HASH        8
#define NUM_CRYPTO_SEED          16
#define NUM_RID                  4

unsigned int g_rom_info_sbc_attr;
unsigned int g_rom_info_sdl_attr;
unsigned int g_hw_sbcen;

unsigned int g_efuse_d0;
unsigned int g_efuse_d3;

struct masp_tag {
    u32 size;
    u32 tag;
    unsigned int rom_info_sbc_attr;
    unsigned int rom_info_sdl_attr;
    unsigned int hw_sbcen;
    unsigned int lock_state;
    unsigned int rid[NUM_RID];
    /*rom_info.m_SEC_KEY.crypto_seed */
    unsigned char crypto_seed[NUM_CRYPTO_SEED];
    unsigned int sbc_pubk_hash[NUM_SBC_PUBK_HASH];
  };
// liujia, UGFDDS-1603, 2023-7-31, end

// camera info.
G_CAM_INFO_S g_cam_info_struct[MAX_DRIVERS] ;
EXPORT_SYMBOL(g_cam_info_struct);

// liujia, UGFDDS-1603, 2023-7-31, sta
int masp_get_from_dts(void)
{
    struct masp_tag *tags;
    struct device_node *np_chosen = NULL;

    np_chosen = of_find_node_by_path("/chosen");
    if (!np_chosen)
    {
        np_chosen = of_find_node_by_path("/chosen@0");
        if (!np_chosen) {
            pr_info("masp_get_from_dts, np_chosen is 0, return.\n");
            return 1;
        }
    }

    tags = (struct masp_tag *)
            of_get_property(np_chosen, "atag,masp", NULL);

	if (!tags)
    {
        pr_info("masp_get_from_dts, tags is 0, return.\n");
        return 1;
    }

    g_rom_info_sbc_attr = tags->rom_info_sbc_attr;
    g_rom_info_sdl_attr = tags->rom_info_sdl_attr;
    g_hw_sbcen = tags->hw_sbcen;

    return 0;
}

int sec_schip_enabled(void)
{
    int ret = 0;

    if (true == g_hw_sbcen) {
        pr_info("sec_schip_enabled, SC\n");

        return 1;
    }

    pr_info("sec_schip_enabled, NSC\n");

    return ret;
}

int sec_usbdl_enabled(void)
{
    switch (g_rom_info_sdl_attr) {
    case ATTR_SUSBDL_ENABLE:
        pr_info("sec_usbdl_enabled, SUSBDL is enabled\n");
        pr_info("sec_usbdl_enabled, 0x%x, SD-FORCE\n", ATTR_SUSBDL_ENABLE);
        return 1;

    /* SUSBDL can't be disabled on security chip */
    case ATTR_SUSBDL_DISABLE:
    case ATTR_SUSBDL_ONLY_ENABLE_ON_SCHIP:
        pr_info("sec_usbdl_enabled, SUSBDL is only enabled on S-CHIP\n");
        if (true == g_hw_sbcen) {
            pr_info("sec_usbdl_enabled, 0x%x, SD-SC\n",
                ATTR_SUSBDL_ONLY_ENABLE_ON_SCHIP);
            return 1;
        }
        pr_info("sec_usbdl_enabled, 0x%x, SD-NSC\n",
            ATTR_SUSBDL_ONLY_ENABLE_ON_SCHIP);
        return 0;

    default:
        pr_info("sec_usbdl_enabled, invalid susbdl config (SD-0x%x)\n",
            g_rom_info_sdl_attr);

        return 1;
	}
}

int sec_boot_enabled(void)
{
    switch (g_rom_info_sbc_attr) {
    case ATTR_SBOOT_ENABLE:
        pr_info("sec_boot_enabled, SBOOT is enabled\n");
        pr_info("sec_boot_enabled, 0x%x, SB-FORCE\n", ATTR_SBOOT_ENABLE);
        return 1;

    /* secure boot can't be disabled on security chip */
    case ATTR_SBOOT_DISABLE:
    case ATTR_SBOOT_ONLY_ENABLE_ON_SCHIP:
        pr_info("sec_boot_enabled, SBOOT is only enabled on S-CHIP\n");
        if (true == g_hw_sbcen) {
            pr_info("sec_boot_enabled, 0x%x, SB-SC\n",
                ATTR_SBOOT_ONLY_ENABLE_ON_SCHIP);

            return 1;
        }

        pr_info("sec_boot_enabled, 0x%x, SB-NSC\n",
                ATTR_SBOOT_ONLY_ENABLE_ON_SCHIP);
        return 0;

    default:
        pr_info("sec_usbdl_enabled, invalid sboot config (SB-0x%x)\n",
                g_rom_info_sbc_attr);

        return 0;
    }
}

int efuse_d0_d3_get_from_dts(void)
{
    int *efuse_d0;
    int *efuse_d3;
    struct device_node *np_chosen = NULL;

    np_chosen = of_find_node_by_path("/chosen");
    if (!np_chosen)
    {
        np_chosen = of_find_node_by_path("/chosen@0");
        if (!np_chosen) {
            pr_info("efuse_d0_d3_get_from_dts, np_chosen is 0, return.\n");
            return 1;
        }
    }

    efuse_d3 = (int *)of_get_property(np_chosen, "efuse_d3", NULL);
    if (efuse_d3) {
        g_efuse_d3 = be32_to_cpu(*efuse_d3);
        pr_info("efuse_d0_d3_get_from_dts, efuse_d3:0x%X", g_efuse_d3);
    }

    efuse_d0 = (int *)of_get_property(np_chosen, "efuse_d0", NULL);
    if (efuse_d0) {
        g_efuse_d0 = be32_to_cpu(*efuse_d0);
        pr_info("efuse_d0_d3_get_from_dts, efuse_d0:0x%X", g_efuse_d0);
    }

    return 0;
}
// liujia, UGFDDS-1603, 2023-7-31, end

#ifdef MODULE
static char __secure_boot_cmdline[COMMAND_LINE_SIZE];
static char *secure_boot_cmdline = __secure_boot_cmdline;

const char *secure_boot_get_cmd(void)
{
    struct device_node *of_chosen = NULL;
    char *bootargs = NULL;

    if (__secure_boot_cmdline[0] != 0)
        return secure_boot_cmdline;

    of_chosen = of_find_node_by_path("/chosen");
    if (of_chosen)
    {
        bootargs = (char *)of_get_property(
                       of_chosen, "bootargs", NULL);
        if (!bootargs)
            pr_err("%s: failed to get bootargs\n", __func__);
        else
        {
            strncpy(__secure_boot_cmdline, bootargs, COMMAND_LINE_SIZE);
            pr_info("%s: bootargs: %s\n", __func__, bootargs);
        }
    }
    else
        pr_err("%s: failed to get /chosen \n", __func__);

    return secure_boot_cmdline;
}
#else
const char *secure_boot_get_cmd(void)
{
    return saved_command_line;
}
#endif

#if IS_ENABLED(CONFIG_T_PRODUCT_INFO)
int get_secure_boot_cb(char *buf, void *args)
{
    // liujia, UGFDDS-1603, 2023-7-31, sta
    int sec_boot = -1;
    int sec_usbdl = -1;
    int sec_schip = -1;

    masp_get_from_dts();

    efuse_d0_d3_get_from_dts();

    sec_boot = sec_boot_enabled();
    sec_usbdl = sec_usbdl_enabled();
    sec_schip = sec_schip_enabled();

    pr_info("get_secure_boot_cb, sec_boot: %d, sec_usbdl: %d, sec_schip: %d, sla: 0x%X, d0: 0x%X\n",
                sec_boot, sec_usbdl, sec_schip, g_efuse_d3, g_efuse_d0);

    return sprintf(buf, "[secboot:(%d) sec_usbdl:(%d) efuse:(%d) sla:(0x%X) d0:(0x%X)]",
                     sec_boot, sec_usbdl, sec_schip, g_efuse_d3, g_efuse_d0);

    /*char secure_boot_str[64] = {0};
    char *ptr = NULL, *ptr_e = NULL;
    char keyword[] = "secure.enable=";
    int size = 0;
    int sec_schip = -1;

    ptr = strstr(secure_boot_get_cmd(), keyword);
    if (ptr != 0)
    {
        ptr_e = strstr(ptr, " ");
        if (ptr_e == 0)
            goto end;

        size = ptr_e - (ptr + strlen(keyword));
        if (size <= 0)
            goto end;
        strncpy(secure_boot_str, ptr + strlen(keyword), size);
        secure_boot_str[size] = '\0';
        pr_notice("%s: secure_boot: %s\n", __func__, secure_boot_str);

        if (!strncmp(secure_boot_str, "yes", strlen("1")))
        {
            sec_schip = 1;
        }
        else if (!strncmp(secure_boot_str, "no", strlen("0")))
        {
            sec_schip = 0;
        }
    }
    return sprintf(buf, "[efuse:(%d)]", sec_schip);
end:
    pr_info("%s: secure_boot = %d\n", __func__, sec_schip);
    return sprintf(buf, "[efuse:(%d)]", sec_schip);*/
    // liujia, UGFDDS-1603, 2023-7-31, end
}
#endif

// for camera info.
int get_camera_info(char *buf, void *arg0)
{
    char *tmp = (char *)arg0;
    int i = 0;
    long resolv = 0;
    int pi = 0;

    if (strcmp(tmp, "main") == 0)
    {
        i = 0;
    }
    else if (strcmp(tmp, "sub") == 0)
    {
        i = 1;
    }
    else if (strcmp(tmp, "main2") == 0)
    {
        i = 2;
    }
    else if (strcmp(tmp, "sub2") == 0)
    {
        i = 3;
    }
    else if (strcmp(tmp, "main3") == 0)
    {
        i = 4;
    }
    else if (strcmp(tmp, "main4") == 0)
    {
        i = 6;
    }
    else
    {
        i = 5;
    }

    resolv = g_cam_info_struct[i].w * g_cam_info_struct[i].h;
    pi = resolv / 1000 / 1000 + (resolv / 1000 / 100 % 10 > 5 ? 1 : 0);

    pr_info("%s:%d - (%s) %s:(%d*%d)\n", __func__, __LINE__, tmp,
            g_cam_info_struct[i].name, g_cam_info_struct[i].w, g_cam_info_struct[i].h);
    if (resolv > 270000 && resolv < 330000)
    {
        return sprintf(buf, "%s [%d*%d] 0.3M",
                       g_cam_info_struct[i].name, g_cam_info_struct[i].w, g_cam_info_struct[i].h);
    }
    else
    {
        return sprintf(buf, "%s [%d*%d] %dM",
                       g_cam_info_struct[i].name, g_cam_info_struct[i].w, g_cam_info_struct[i].h, pi);
    }
}
EXPORT_SYMBOL(get_camera_info);

int set_camera_info(char *sensor_name, int sensor_id, int invokeSocketIdx, long SensorFullWidth, long SensorFullHeight)
{
    sprintf(g_cam_info_struct[sensor_id].name, "%s", sensor_name);
    g_cam_info_struct[sensor_id].id = invokeSocketIdx;
    g_cam_info_struct[sensor_id].w = SensorFullWidth;
    g_cam_info_struct[sensor_id].h = SensorFullHeight;
    return 0;
}
EXPORT_SYMBOL(set_camera_info);

/*******************************************************************************************/
// flash info.(mtk,sprd platform)
int get_mmc_chip_info(char *buf, void *arg0)
{
    struct mmc_card *card = (struct mmc_card *)arg0;
    char tempID[64] = "";
    char vendorName[16] = "";
    char romsize[8] = "";
    char ramsize[8] = "";
    struct sysinfo si;
    si_meminfo(&si);
    if (si.totalram > 1572864)                 // 6G = 1572864  (256 *1024)*6
        strcpy(ramsize, "8G");
    else if (si.totalram > 1048576)           // 4G = 786432    (256 *1024)*4
        strcpy(ramsize, "6G");
    else if (si.totalram > 786432)           // 3G = 786432     (256 *1024)*3
        strcpy(ramsize, "4G");
    else if (si.totalram > 524288)          // 2G = 524288  (256 *1024)*2
        strcpy(ramsize, "3G");
    else if (si.totalram > 262144)              // 1G = 262144      (256 *1024)     4K page size
        strcpy(ramsize, "2G");
    else if (si.totalram > 131072)              // 512M = 131072        (256 *1024/2)   4K page size
        strcpy(ramsize, "1G");
    else
        strcpy(ramsize, "512M");

    if (card->ext_csd.sectors > 134217728)
        strcpy(romsize, "128G");
    else if (card->ext_csd.sectors > 67108864)  // 67108864 = 32G *1024*1024*1024 /512            512 page
        strcpy(romsize, "64G");
    else if (card->ext_csd.sectors > 33554432) // 33554432 = 16G *1024*1024*1024 /512            512 page
        strcpy(romsize, "32G");
    else if (card->ext_csd.sectors > 16777216) // 16777216 = 8G *1024*1024*1024 /512            512 page
        strcpy(romsize, "16G");
    else if (card->ext_csd.sectors > 8388608) // 8388608 = 4G *1024*1024*1024 /512            512 page
        strcpy(romsize, "8G");
    else
        strcpy(romsize, "4G");

    memset(tempID, 0, sizeof(tempID));
    sprintf(tempID, "%08x ", card->raw_cid[0]);
    pr_info("FlashID is %s, totalram= %ld, emmc_capacity =%d\n", tempID, si.totalram, card->ext_csd.sectors);

    if (strncasecmp((const char *)tempID, "90", 2) == 0)          // 90 is OEMid for Hynix
        strcpy(vendorName, "Hynix");
    else if (strncasecmp((const char *)tempID, "15", 2) == 0)       // 15 is OEMid for Samsung
        strcpy(vendorName, "Samsung");
    else if (strncasecmp((const char *)tempID, "45", 2) == 0)       // 45 is OEMid for Sandisk
        strcpy(vendorName, "Sandisk");
    else if (strncasecmp((const char *)tempID, "70", 2) == 0)       // 70 is OEMid for Kingston
        strcpy(vendorName, "Kingston");
    else if (strncasecmp((const char *)tempID, "88", 2) == 0)       // 88 is OEMid for Foresee
        strcpy(vendorName, "Foresee");
    else if (strncasecmp((const char *)tempID, "f4", 2) == 0)       // f4 is OEMid for Biwin
        strcpy(vendorName, "Biwin");
    else if (strncasecmp((const char *)tempID, "d6", 2) == 0)           // d6 is OEMid for Foresee
        strcpy(vendorName, "Foresee");
    else if (strncasecmp((const char *)tempID, "ec", 2) == 0)           // ec is OEMid for Rayson
        strcpy(vendorName, "Rayson");
    else if (strncasecmp((const char *)tempID, "04", 2) == 0)           // 04 is OEMid for Foresee
        strcpy(vendorName, "Foresee");
    else if (strncasecmp((const char *)tempID, "df", 2) == 0)           // df is OEMid for Foresee
        strcpy(vendorName, "Foresee");
    else if (strncasecmp((const char *)tempID, "32", 2) == 0)           // 32 is OEMid for Rayson
        strcpy(vendorName, "Rayson");
    else
        strcpy(vendorName, "Unknown");

    memset(tempID, 0, sizeof(tempID));
    sprintf(tempID, "%s_%s+%s,prv:%02x,life:%02x,%02x", vendorName, romsize, ramsize, card->cid.prv,
            card->ext_csd.device_life_time_est_typ_a, card->ext_csd.device_life_time_est_typ_b);

    return sprintf(buf, "%s", tempID);
}
EXPORT_SYMBOL(get_mmc_chip_info);
