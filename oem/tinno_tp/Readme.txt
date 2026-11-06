TINNO tp share folder statement‌
======================================

项目需求开关添加示例:

1.项目的config 文件
CONFIG_TINNO_TOUCHSCREEN_PEN =y


2../Kconfig
config TINNO_TOUCHSCREEN_PEN
    tristate "TOUCH PEN"
    default N
    help
      Say Y here if enable touch pen.
      If unsure, say N.


3.tp驱动代码

#if IS_ENABLED(CONFIG_TINNO_TOUCHSCREEN_PEN)
#define FTS_PEN_EN                              1
#else
#define FTS_PEN_EN                              0
#endif

4.TBC
