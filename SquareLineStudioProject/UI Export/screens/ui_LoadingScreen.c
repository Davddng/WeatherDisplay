// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.3
// LVGL version: 8.3.3
// Project name: SquareLine_Project

#include "../ui.h"

void ui_LoadingScreen_screen_init(void)
{
ui_LoadingScreen = lv_obj_create(NULL);
lv_obj_clear_flag( ui_LoadingScreen, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Spinner1 = lv_spinner_create(ui_LoadingScreen,1000,90);
lv_obj_set_width( ui_Spinner1, 300);
lv_obj_set_height( ui_Spinner1, 300);
lv_obj_set_align( ui_Spinner1, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_Spinner1, LV_OBJ_FLAG_CLICKABLE );    /// Flags
lv_obj_set_style_arc_color(ui_Spinner1, lv_color_hex(0x4040FF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_Spinner1, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_arc_color(ui_Spinner1, lv_color_hex(0x00D6FF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_Spinner1, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_arc_width(ui_Spinner1, 20, LV_PART_INDICATOR| LV_STATE_DEFAULT);

ui_Label1 = lv_label_create(ui_LoadingScreen);
lv_obj_set_width( ui_Label1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label1, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label1,"Loading...\nPlease wait.");
lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label1, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

}
