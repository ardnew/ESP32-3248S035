#include "main.hpp"

uint8_t wheel(lv_color32_t &rgb, uint8_t const i, int8_t const step = 1) {
  uint8_t curr = i;
  uint8_t next = i + step;
  curr = 0xFF - curr;
  if (curr < 0x55) {
    rgb.red   = 0x03 * curr;
    rgb.green = 0xFF - curr * 0x03;
    rgb.blue  = 0x00;
  }
  else if (curr < 0xAA) {
    curr -= 0x55;
    rgb.red   = 0xFF - curr * 0x03;
    rgb.green = 0x00;
    rgb.blue  = 0x03 * curr;
  }
  else {
    curr -= 0xAA;
    rgb.red   = 0x00;
    rgb.green = 0x03 * curr;
    rgb.blue  = 0xFF - curr * 0x03;
  }
  return next;
}

template <typename T = msecu32_t::rep, T Freq = 20>
bool can_refresh(T const now = msecu32().count()) {
  if constexpr (Freq != 0) {
    static T last = 0;
    if (now - last < Freq) {
      return false;
    }
    last = now;
  }
  return true;
}

// The Arduino API requires a setup() and loop(). These are where you initiate
// and refresh the GUI elements and all other hardware peripherals.
void setup() {
  target.init();
}

void loop() {
  static uint8_t i_wheel = 0;
  if (can_refresh()) {
    // The individual hardware peripherals can be accessed via template
    // parameter on method hw<T>(), where T is the peripheral type:
    lv_color32_t rgb = target.hw<bsp::rgb_type>().get();
    i_wheel = wheel(rgb, i_wheel);

    target.hw<bsp::rgb_type>().set(rgb);
    target.update();
  }
}

// If using the lvgl logging system (LV_USE_LOG), you must:
//   1. Define a tpc_type::log_type function, and
//   2. Assign it to the static log member of the tpc_type class.
// Both steps are shown here:
#if (LV_USE_LOG)
template<> const bsp::tpc_type::log_type bsp::tpc_type::log =
  [](lv_log_level_t ll, const char *msg) {
    static const char *pre[_LV_LOG_LEVEL_NUM] = {
      "[-] ", // [0] LV_LOG_LEVEL_TRACE
      "[=] ", // [1] LV_LOG_LEVEL_INFO
      "[~] ", // [2] LV_LOG_LEVEL_WARN
      "[!] ", // [3] LV_LOG_LEVEL_ERROR
      "[+] "  // [4] LV_LOG_LEVEL_USER
    };
    unsigned level = static_cast<unsigned>(ll);
    if (level < LV_LOG_LEVEL_NONE) {
      Serial.print(pre[level]);
    }
    Serial.println(msg);
  };
#endif
