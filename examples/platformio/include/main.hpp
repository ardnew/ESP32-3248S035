#pragma once

#include <ESP323248S035.hpp>

// The ESP32-3248S035 library defines View as a pure virtual class, which acts
// as an interface between the library and application GUI code.
//
// We must define a concrete implementation of View to act as the root window
// of the target device's GUI. The content of View is application-defined.
//
// NOTE: Do NOT make any lvgl API calls from the View constructor. These must be
//       performed in the View::init() method.
class Main: public bsp::View {
public:
  Main() = default;
  ~Main() = default;

  bool init(lv_obj_t *root) override {
    if (nullptr == root) {
      // Use the default screen if no root view provided.
      root = lv_scr_act();
    }
    lv_obj_t *label = lv_label_create(root);
    lv_obj_center(label);
    lv_label_set_text(label, "Hello, Arduino!");
    return true;
  }

  void update(msecu32_t const now) override {
    // Update any subviews here.
  }

  std::string title() override {
    return "Main";
  }
};

Main root;
//bsp::ESP323248S035C target(root); // C++17 can deduce template parameter.
bsp::ESP323248S035C<Main> target(root); // Pre-C++17 requires explicit type.
