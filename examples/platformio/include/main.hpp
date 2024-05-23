#pragma once

#include <esp323248s035.hpp>

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

public:
  Main(bsp::LCD *lcd): _lcd(lcd) {}
  ~Main() = default;

  esp_err_t init(lv_obj_t *root) override {
    if (nullptr == root) {
      // Use the active screen if no root object is provided.
      root = lv_scr_act();
    }
    lv_obj_set_style_bg_color(root, lv_color_make(0xFF, 0, 0), 0);
    lv_obj_t *label = lv_label_create(root);
    lv_obj_center(label);
    lv_label_set_text(label, "Hello, Arduino!");
    return ESP_OK;
  }

  void update(msecu32_t const now) override {
    std::size_t queued, flushed;
    {
     _lcd->lock();
     queued = _lcd->queued;
     flushed = _lcd->flushed;
    }
    printf("update(%u): queued=%lu flushed=%lu\n", now.count(), queued, flushed);
    // lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(queued & 0xFF, 0, 0), 0);
  }

  std::uint32_t refresh_hz() const noexcept override {
    return 10; // milliseconds
  }

  std::string title() override {
    return "Main";
  }
};
