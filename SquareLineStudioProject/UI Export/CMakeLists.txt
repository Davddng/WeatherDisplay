SET(SOURCES components/ui_comp_clockedge.c
    components/ui_comp_dials.c
    components/ui_comp.c
    screens/ui_LoadingScreen.c
    screens/ui_Screen1.c
    screens/ui_Screen2.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_caution_png.c
    images/ui_img_hourhand_png.c
    images/ui_img_minutehand_png.c
    images/ui_img_secondhand_png.c
    images/ui_img_clockcenter_png.c)

add_library(ui ${SOURCES})
