#pragma once

#include <lvgl.h>
#include <esp323248s035.hpp>

extern "C" void app_main(void);

extern "C" void btn_event_cb(lv_event_t *e) {
  log_trace("button event");
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *btn = static_cast<lv_obj_t *>(lv_event_get_target(e));
  if (code == LV_EVENT_CLICKED) {
    static uint8_t cnt = 0;
    cnt++;

    /*Get the first child of the button which is the label and change its text*/
    lv_obj_t *lbl = lv_obj_get_child(btn, 0);
    lv_label_set_text_fmt(lbl, "Button: %d", cnt);
  }
}

// The ESP32-3248S035 library defines View as a pure virtual class, which acts
// as an interface between the library and application GUI code.
//
// We must define a concrete implementation of View to act as the root window
// of the target device's GUI. The content of View is application-defined.
//
// NOTE: Do NOT make any lvgl API calls from the View constructor. These must be
//       performed in the View::init() method.
class Main: public bsp::View {
protected:
  bsp::LCD *_lcd = nullptr;

  lv_obj_t *_btn;
  lv_obj_t *_lbl;

public:
  Main(bsp::LCD *lcd): _lcd(lcd) {}
  ~Main() = default;

  esp_err_t init(lv_obj_t *root) override {

    lv_display_set_rotation(_lcd->display(), LV_DISPLAY_ROTATION_90);

    _btn = lv_button_create(root);     /*Add a button the current screen*/
    lv_obj_set_pos(_btn, 10, 10);                            /*Set its position*/
    lv_obj_set_size(_btn, 120, 50);                          /*Set its size*/
    lv_obj_add_event_cb(_btn, btn_event_cb, LV_EVENT_CLICKED, NULL);           /*Assign a callback to the button*/

    _lbl = lv_label_create(_btn);          /*Add a label to the button*/
    lv_label_set_text(_lbl, "Button");                     /*Set the labels text*/
    lv_obj_center(_lbl);

    return ESP_OK;
  }

  esp_err_t update(TickType_t const now) override {
    // printf("update(%lld)\n", now);
    // lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(static_cast<std::uint32_t>(now)), 0);
    log_trace("update main view (ticks() = %lu)", now);
    return ESP_OK;
  }

  inline TickType_t refresh_rate() const noexcept override {
    return pdMS_TO_TICKS((1000ms).count());
  }

  inline std::string title() override {
    return "Main";
  }
};
