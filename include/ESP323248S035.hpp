#pragma once

// Arduino-ESP32 core
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Ticker.h>
// ESP-IDF SDK
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// External libraries
#include <lvgl.h>
#include <cronos.hpp>
// C++ stdlib
#include <chrono>
#include <mutex>
#include <cstddef>
#include <utility>

namespace bsp {

// The sole template parameter msecu32_t N defines the frequency (miilliseconds)
// of which the given function func is called.
//
// Note that func is not required to run in the same, fixed time duration every
// iteration. The intervening delay before func is called again will be recalcu-
// lated to ensure consistent frequency (this is the purpose and responsibility
// of FreeRTOS function xTaskDelayUntil).
//
// The only constraint on N is that it is greater than the maximum _possible_
// duration of func. Macro TASK_SCHED_GREEDY configures the behavior when this
// constraint is not met:
//
//   If func takes longer than N milliseconds and:
//     Defined(TASK_SCHED_GREEDY):
//       The subsequent func is invoked immediately without delay.
//       If this condition continues, it will starve the IDLE task and trigger
//       the IWDT to reset the system. This will ensure func runs as frequently
//       as possible, but it might cause system resets.
//     NotDefined(TASK_SCHED_GREEDY):
//       The subsequent func is deferred to the following period.
//       This means the func whose deadline was missed will never execute, which
//       reduces the frequency of func but gives the IDLE task more opportunity
//       to run and reset the IWDT.
//       This behavior should improve system stability.
//       Therefore, it is the default.
//
template <const uint32_t N> void run(std::function<void(void)> func) {
  // xTaskDelayUntil arguments are expressed in units of system "ticks".
  // Conventionally, 1 tick = 1 millisecond regardless of CPU frequency.
  static constexpr TickType_t freq = N / portTICK_PERIOD_MS;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
#ifndef TASK_SCHED_GREEDY
    if /* conditionally run func() iff time has elapsed */
#endif
    (xTaskDelayUntil(&last, freq))
    { func(); }
  }
}

using pin_type = int8_t;

// Atomic provides a mutual exclusion mechanism for shared resources.
template <typename MutexType = std::mutex,
  typename GuardType = std::unique_lock<MutexType>>
struct Atomic: MutexType {
  using type = Atomic;
  using mutex_type = MutexType;
  using guard_type = GuardType;
  // Take ownership of this mutex until the returned value is destroyed.
  virtual inline guard_type make() { return guard_type(*this); }
};

struct Periodic { virtual void update(msecu32_t const) = 0; };
struct Controller: Periodic { virtual bool init() = 0; };

struct View : Periodic {
  virtual bool init(lv_obj_t *) = 0;
  virtual std::string title() = 0;
};

template <typename T = SPIClass> struct SPIImpl;
template <> struct SPIImpl<SPIClass> : SPIClass {
  using type = SPIClass;
  using type::type;
  inline bool init() { type::begin(); return true; }
};

template <typename T = TwoWire> struct I2CImpl;
template <> struct I2CImpl<TwoWire> : TwoWire {
  using type = TwoWire;
  using type::type;
  inline bool init() { return type::begin(); }
};

template <typename AtomicType = Atomic<>,
  typename SPIType = SPIImpl<>, typename I2CType = I2CImpl<>>
class TPC_LCD: public Controller, AtomicType, SPIType, I2CType {
public:
  using type        = TPC_LCD;
  using atomic_type = AtomicType;
  using spi_type    = SPIType;
  using i2c_type    = I2CType;

  // C callback signatures required by lvgl
  //  (note: omit tick() because it's a static method we can use directly)
  struct Callback {
    using log_type = void (*)(lv_log_level_t, const char *);
    using flush_type = void (*)(lv_display_t *, const lv_area_t *, uint8_t *);
    using read_type = void (*)(lv_indev_t *, lv_indev_data_t *);
    inline static type *instance; // must be public for access from C context
    static void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *data) {
      instance->flush(disp, area, data);
    }
    static void read(lv_indev_t *drv, lv_indev_data_t *data) {
      instance->read(drv, data);
    }
  };

  using log_type = typename Callback::log_type;
  using flush_type = typename Callback::flush_type;
  using read_type = typename Callback::read_type;

#if (LV_USE_LOG)
  // User is required to provide log whenever the LV_USE_LOG macro is defined.
  // It will be registered with lvgl automatically in method TPC_LCD::init().
  static const log_type log;
#endif

protected:
  struct __attribute__((packed)) point_t {
    // attributes of an individual touch event, this structure is repeated
    // contiguously in the following memory-mapped registers, which implements
    // 5-point multi-touch tracking (in the hardware).
    // however, the LCD graphics library (lvgl or TFT_eSPI/LoyvanGFX) currently
    // supports single-touch only.
    //   1. 0x814F—0x8156
    //   2. 0x8157—0x815E
    //   3. 0x815F—0x8166
    //   4. 0x8167—0x816E
    //   5. 0x816F—0x8176
    uint8_t track;
    uint16_t x;
    uint16_t y;
    uint16_t area;
    // 1 pad byte makes 64 bits in (packed) struct
    uint8_t _u8[1]; // Value is irrelevant
  };

  enum class cmd_t : uint8_t {
    swreset = 0x01, // Software Reset
    slpin = 0x10,   // Sleep in
    slpout = 0x11,  // Sleep out
    noron = 0x13,   // Normal Display Mode On
    invoff = 0x20,  // Display Inversion Off
    dispon = 0x29,  // Display On
    caset = 0x2A,   // Column Address Set
    raset = 0x2B,   // Row Address Set
    ramwr = 0x2C,   // Memory Write
    madctl = 0x36,  // Memory Data Access Control
    colmod = 0x3A,  // Interface Pixel Format
    pgc = 0xE0,     // Positive Gamma Control
    ngc = 0xE1,     // Negative Gamma Control
    cscon = 0xF0,   // Command Set wqaControl
  };
  enum class madctl_t : uint8_t {
    my = 0x80,  // Row Addr Order: 0=Inc(↓), 1=Dec(↑)
    mx = 0x40,  // Column Addr Order: 0=Inc(→), 1=Dec(←)
    mv = 0x20,  // Row/Col exchange: 0=Norm, 1=Exchange
    ml = 0x10,  // Vertical Refresh Order
    bgr = 0x08, // Subpixel rendering: BGR order
    mh = 0x10,  // Horizontal Refresh Order
    rgb = 0x00, // Subpixel rendering: RGB order
    // MADCTL rotation bitmasks
    // We can conveniently translate these bitmasks to unique values 0..3,
    // AND preserve the ordering for computing degrees of rotation, as follows:
    tall = my,                    // ((128 >> 5) & 3) * 90 == 0 * 90 =   0°
    wide = mv,                    //  ((32 >> 5) & 3) * 90 == 1 * 90 =  90°
    tall_inverted = mx,           //  ((64 >> 5) & 3) * 90 == 2 * 90 = 180°
    wide_inverted = my | mv | mx, // ((224 >> 5) & 3) * 90 == 3 * 90 = 270°
  };
  // Like the enum definitions above, these color mode constants are used by
  // the MIPI TFT init sequence (n.b., unrelated to lvgl (LV_COLOR_*)).
  enum class colmod_t : uint8_t {
    rgb16 = 0x50,
    ctrl16 = 0x05,
    rgb656 = rgb16 | ctrl16,
  };

  // GPIO pins
  static pin_type constexpr _pin_sdi = 12U; // SPI MISO
  static pin_type constexpr _pin_sdo = 13U; // SPI MOSI
  static pin_type constexpr _pin_sck = 14U; // SPI SCLK
  static pin_type constexpr _pin_sel = 15U; // SPI SS/CS
  static pin_type constexpr _pin_sdc = 2U;  // LCD DC/RS
  static pin_type constexpr _pin_blt = 27U; // delicious BLT
  static pin_type constexpr _pin_scl = 32U; // I²C SCL
  static pin_type constexpr _pin_sda = 33U; // I²C SDA
  static pin_type constexpr _pin_int = 21U; // TPC INT
  static pin_type constexpr _pin_rst = 25U; // TPC RST
  // SPI interfaces
  static uint8_t constexpr _spi_bus = 2U;          // HSPI
  static uint32_t constexpr _bus_freq = 80000000U; // 80 MHz
  static uint8_t constexpr _word_ord = SPI_MSBFIRST;
  static uint8_t constexpr _sig_mode = SPI_MODE0;
  // I²C interfaces
  static uint8_t constexpr _i2c_bus = 1U;    // Wire1
  static uint8_t constexpr _dev_addr = 0x5D; // I²C touch device address
  // Backlight PWM
  static uint8_t constexpr _pwm_blt_chan = 12U;
  static uint32_t constexpr _pwm_blt_freq = 5000U; // 5 kHz
  static uint8_t constexpr _pwm_blt_bits = 8U;
  static uint8_t constexpr _pwm_blt_hres = (1U << _pwm_blt_bits) - 1U;
  // TFT configuration
  static msecu32_t constexpr _tft_refresh = msecu32_t{10}; // milliseconds
  static uint16_t constexpr _tft_width = 320U;  // Physical dimensions, does
  static uint16_t constexpr _tft_height = 480U; // not consider rotations.
  static uint8_t constexpr _tft_colmod = static_cast<uint8_t>(colmod_t::rgb656);
  static uint8_t constexpr _tft_bpp = 16; // bits per pixel
  static uint8_t constexpr _tft_pxsize = (_tft_bpp + 7U) >> 3U; // bytes
  static size_t constexpr _tft_bufcount = 2U;
  static size_t constexpr _tft_bufline = 10U;
  static size_t constexpr _tft_bufsize = _tft_width * _tft_bufline * _tft_pxsize;
  static uint8_t constexpr _tft_subpixel = static_cast<uint8_t>(madctl_t::rgb);
  static uint8_t constexpr _tft_aspect = static_cast<uint8_t>(madctl_t::wide_inverted);
  // Touch MMIO
  static uint16_t constexpr _reg_prod_id = 0x8140; // product ID (byte 1/4)
  static size_t constexpr _size_prod_id = 4U;      // bytes
  static uint16_t constexpr _reg_stat = 0x814E;    // buffer/track status
  static uint16_t constexpr _reg_base_point = 0x814F;
  static size_t constexpr _touch_max = 5U; // simultaneous touch points

  // C callback signature of timebase required by lvgl.
  static inline void tick() { lv_tick_inc(_tft_refresh.count()); }

  void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *data) {
    tx(cmd_t::caset,
      static_cast<uint8_t>(area->x1 >> 8),
      static_cast<uint8_t>(area->x1),
      static_cast<uint8_t>(area->x2 >> 8),
      static_cast<uint8_t>(area->x2));
    tx(cmd_t::raset,
      static_cast<uint8_t>(area->y1 >> 8),
      static_cast<uint8_t>(area->y1),
      static_cast<uint8_t>(area->y2 >> 8),
      static_cast<uint8_t>(area->y2));

    px(cmd_t::ramwr, data, lv_area_get_size(area) * pxsize());
    lv_display_flush_ready(disp);
  }

  void read(lv_indev_t *drv, lv_indev_data_t *data) {
    static int16_t _x = 0, _y = 0;
    point_t pt;
    switch (touch_count()) {
    case 1:
      if (map(&pt, 1)) {
        data->state = LV_INDEV_STATE_PR;
        _x = data->point.x = pt.x;
        _y = data->point.y = pt.y;
      }
      break;
    default:
      data->point.x = _x;
      data->point.y = _y;
      data->state = LV_INDEV_STATE_REL;
      break;
    }
  }

  void px(cmd_t const code, const uint8_t data[], size_t const size) {
    digitalWrite(_pin_sdc, LOW); // D/C ⇒ command
    spi_type::beginTransaction(SPISettings(_bus_freq, _word_ord, _sig_mode));
    digitalWrite(_pin_sel, LOW); // C/S ⇒ enable
    spi_type::write(static_cast<uint8_t>(code));
    if (size > 0) {
      digitalWrite(_pin_sdc, HIGH); // D/C ⇒ data
      spi_type::writeBytes(data, size);
    }
    digitalWrite(_pin_sel, HIGH); // C/S ⇒ disable
    spi_type::endTransaction();
  }

  template <class... E> void tx(cmd_t const code, E... e) {
    constexpr size_t size = sizeof...(e);
    uint8_t data[size] = {static_cast<uint8_t>(e)...};
    px(code, data, size);
  }

  inline bool r16(uint16_t const reg) {
    i2c_type::beginTransmission(_dev_addr);
    return i2c_type::write(static_cast<uint8_t>(reg >> 8)) && // MSB (first)
           i2c_type::write(static_cast<uint8_t>(reg & 0xFF)); // LSB
  }

  inline bool tx(uint16_t const reg, uint8_t *const buf, size_t const len) {
    size_t sent = 0U;
    if (r16(reg)) {
      sent = i2c_type::write(buf, len);
    }
    i2c_type::endTransmission();
    return sent == len;
  }

  inline bool rx(uint16_t const reg, uint8_t *const buf, size_t const len) {
    uint8_t *dst = buf;
    size_t rem = 0U;
    bool ok = r16(reg);
    i2c_type::endTransmission(false);
    if (ok && (len == (rem = i2c_type::requestFrom(_dev_addr, len)))) {
      while (i2c_type::available() && rem--) {
        *(dst++) = i2c_type::read();
      }
    }
    return rem == 0 && dst != buf;
  }

  inline bool rx(uint16_t const reg, point_t *const buf, size_t const len) {
    return rx(reg, (uint8_t *)buf, len);
  }

  // template <unsigned N = 0>
  // static inline constexpr uint16_t track() {
  //   return _reg_base_point + N * sizeof(point_t) + offsetof(point_t, track);
  // }

  template <uint16_t R = _tft_aspect>
  inline bool map(point_t *const pt, size_t count) {
    bool ok = rx(_reg_base_point, pt, sizeof(point_t) * count);
    ((std::function<void()>[]){
      [pt, count]() {
        for (uint8_t i = 0; i < count; ++i) {
          pt[i].x = width() - pt[i].x;
          pt[i].y = height() - pt[i].y;
        }
      },
      [pt, count]() {
        uint16_t swap;
        for (uint8_t i = 0; i < count; ++i) {
          swap = pt[i].x;
          pt[i].x = pt[i].y;
          pt[i].y = height() - swap;
        }
      },
      [pt, count]() {
        for (uint8_t i = 0; i < count; ++i) {
          pt[i].x = pt[i].x;
          pt[i].y = pt[i].y;
        }
      },
      [pt, count]() {
        uint16_t swap;
        for (uint8_t i = 0; i < count; ++i) {
          swap = pt[i].x;
          pt[i].x = width() - pt[i].y;
          pt[i].y = swap;
        }
      }}[(R >> 5) & 3])();
    return ok;
  }

  inline size_t touch_count() {
    static uint8_t stat[1] = {0};
    size_t count = 0;
    if (rx(_reg_stat, stat, sizeof(stat))) {
      count = stat[0];
    }
    memset(stat, 0, sizeof(stat));
    if (((count & 0x80) > 0) && ((count & 0x0F) < _touch_max)) {
      tx(_reg_stat, stat, sizeof(stat));
      count &= 0x0F;
    }
    return count;
  }

private:
  static inline lv_display_t *_disp;
  static inline lv_indev_t *_inpt;
  static inline uint8_t _fb[_tft_bufcount][_tft_bufsize] __attribute__((aligned(4)));

  Ticker _tick;
  View &_view;

public:
  TPC_LCD() = delete; // display driver must have a root view!
  explicit TPC_LCD(View &root, log_type log = nullptr)
      : spi_type(_spi_bus), i2c_type(_i2c_bus), _view(root) {
    Callback::instance = this;
  }
  virtual ~TPC_LCD() {}

  bool init() override {
#if (LV_USE_LOG)
    lv_log_register_print_cb(log);
#endif

    lv_init();
    spi_type::begin(_pin_sck, _pin_sdi, _pin_sdo);

    pinMode(_pin_sdc, OUTPUT); // D/C
    pinMode(_pin_sel, OUTPUT); // CS
    digitalWrite(_pin_sel, HIGH);

    pinMode(_pin_blt, OUTPUT); // LITE (PWM)
    ledcSetup(_pwm_blt_chan, _pwm_blt_freq, _pwm_blt_bits);
    ledcAttachPin(_pin_blt, _pwm_blt_chan);

    // ST7796 initialization sequence
    tx(cmd_t::swreset);
    delay(100);
    tx(cmd_t::cscon, 0xC3); // Enable extension command 2 part I
    tx(cmd_t::cscon, 0x96); // Enable extension command 2 part II
    tx(cmd_t::colmod, _tft_colmod); // 16 bits R5G6B5
    tx(cmd_t::madctl, _tft_aspect | _tft_subpixel);
    tx(cmd_t::pgc, 0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B);
    tx(cmd_t::ngc, 0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B);
    tx(cmd_t::cscon, 0x3C); // Disable extension command 2 part I
    tx(cmd_t::cscon, 0x69); // Disable extension command 2 part II
    tx(cmd_t::invoff); // Inversion off
    tx(cmd_t::noron);  // Normal display on
    tx(cmd_t::slpout); // Out of sleep mode
    tx(cmd_t::dispon); // Main screen turn on

    set_backlight(_pwm_blt_hres);

    _disp = lv_display_create(width(), height());
    // Callback<void(lv_display_t *, const lv_area_t *, uint8_t *)>::fn =
    //   std::bind(&type::flush, this,
    //   std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    // lv_display_set_flush_cb(_disp, static_cast<flush_t>(
    //   Callback<void(lv_display_t *, const lv_area_t *, uint8_t *)>::callback));
    //auto flush = static_cast<flush_t>(std::bind(&type::flush, this,
    //  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    lv_display_set_flush_cb(_disp, Callback::flush);
    lv_display_set_buffers(_disp, _fb[0], _fb[1], size(), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_obj_clean(lv_scr_act());

    // ---------------------------------------------------------------------------
    //  TODO: calibrate touch?
    // ---------------------------------------------------------------------------

    bool ok;
    if ((ok = i2c_type::begin(_pin_sda, _pin_scl))) {
      static uint8_t pid[_size_prod_id];
      ok = rx(_reg_prod_id, pid, sizeof(pid));
    }

    _inpt = lv_indev_create();
    // Callback<void(lv_indev_t *, lv_indev_data_t *)>::fn =
    //   std::bind(&type::read, this,
    //   std::placeholders::_1, std::placeholders::_2);
    lv_indev_set_type(_inpt, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(_inpt, Callback::read);
    // lv_indev_set_read_cb(_inpt, static_cast<read_t>(
    //   Callback<void(lv_indev_t *, lv_indev_data_t *)>::callback));

    if ((ok = ok && _view.init(lv_scr_act()))) {
      _tick.attach_ms(_tft_refresh.count(), tick);
    }

    return ok;
  }
  void update(msecu32_t const now) override {
    lv_timer_handler();
    _view.update(now);
  }

  View &view() { return _view; }

  template <uint8_t C = _pwm_blt_chan>
  static void set_backlight(uint16_t duty) {
    ledcWrite(C, duty);
  }

  static constexpr size_t depth(void) noexcept { return _tft_bpp; }
  static constexpr size_t pxsize(void) noexcept { return _tft_pxsize; }
  static constexpr size_t size(void) noexcept { return _tft_bufsize; }
  static constexpr size_t buffers(void) noexcept { return _tft_bufcount; }

  static constexpr size_t wide(void) noexcept {
    return (_tft_aspect & static_cast<uint8_t>(madctl_t::mv)) > 0; // transpose?
  }
  static constexpr size_t width(void) noexcept {
    return wide() ? _tft_height : _tft_width;
  }
  static constexpr size_t height(void) noexcept {
    return wide() ? _tft_width : _tft_height;
  }
};

template <typename AtomicType = Atomic<>>
class RGB_PWM: Controller, AtomicType { // RGB 3-pin analog LR
public:
  using type        = RGB_PWM;
  using atomic_type = AtomicType;

protected:
  static pin_type const _pin_r = 4U;
  static pin_type const _pin_g = 16U;
  static pin_type const _pin_b = 17U;

  static uint8_t const _pwm_r_chan = 13U;
  static uint8_t const _pwm_g_chan = 14U;
  static uint8_t const _pwm_b_chan = 15U;

  static uint32_t const _pwm_freq = 5000U; // 5 kHz
  static uint8_t const _pwm_bits = 8U;
  static uint8_t const _pwm_hres = (1U << _pwm_bits) - 1U;

  lv_color32_t _rgb, _set; // _rgb is last set value, _set is buffer for next

public:
  RGB_PWM() {
  }
  bool init() override {
    // Red
    pinMode(_pin_r, OUTPUT);
    digitalWrite(_pin_r, true);
    ledcSetup(_pwm_r_chan, _pwm_freq, _pwm_bits);
    ledcAttachPin(_pin_r, _pwm_r_chan);
    // Green
    pinMode(_pin_g, OUTPUT);
    digitalWrite(_pin_g, true);
    ledcSetup(_pwm_g_chan, _pwm_freq, _pwm_bits);
    ledcAttachPin(_pin_g, _pwm_g_chan);
    // Blue
    pinMode(_pin_b, OUTPUT);
    digitalWrite(_pin_b, true);
    ledcSetup(_pwm_b_chan, _pwm_freq, _pwm_bits);
    ledcAttachPin(_pin_b, _pwm_b_chan);
    set({});
    update(msecu32_t{0});
    return true;
  }
  void update(msecu32_t const now) override {
    if (now.count() > 0) {
      auto scope = atomic_type::make();
      if (lv_color32_eq(_rgb, _set)) {
        return;
      }
    }
    write(_set);
  }
  lv_color32_t get() noexcept {
    auto scope = atomic_type::make();
    return _rgb;
  }
  void set(lv_color32_t const rgb) noexcept {
    auto scope = atomic_type::make();
    _set = rgb;
  }
  void write(lv_color32_t const rgb) noexcept {
    {
      auto scope = atomic_type::make();
      _rgb = rgb;
    }
    ledcWrite(_pwm_r_chan, _pwm_hres - rgb.red);
    ledcWrite(_pwm_g_chan, _pwm_hres - rgb.green);
    ledcWrite(_pwm_b_chan, _pwm_hres - rgb.blue);
  }
};

template <typename AtomicType = Atomic<>>
class AMP_PWM: Controller, AtomicType { // Autio amplifier
public:
  using type        = AMP_PWM;
  using atomic_type = AtomicType;

protected:
  static pin_type const _pin = 26U;

public:
  bool init() override {
    pinMode(_pin, INPUT); // High impedence
    update(msecu32_t{0});
    return true;
  }
  void update(msecu32_t const now) override {
  }
  void set(uint32_t const hz, msecu32_t const ms) {
    tone(_pin, hz, ms.count());
  }
  void mute() {
    noTone(_pin);
  }
};

template <typename AtomicType = Atomic<>>
class CDS_ADC: Controller, AtomicType { // Light sensitive photo-resistor
public:
  using type        = CDS_ADC;
  using atomic_type = AtomicType;

protected:
  static adc_attenuation_t const _att = ADC_0db; // 0dB (1.0x): 0~800mV

  static pin_type const _pin = 34U; // A0

  int _val; // _val is last read value, _get is buffer for next

public:
  bool init() override {
    analogSetAttenuation(_att); // 0dB(1.0x) 0~800mV
    pinMode(_pin, INPUT);
    update(msecu32_t{0});
    return true;
  }
  void update(msecu32_t const now) override {
    read();
  }
  int get() {
    auto scope = atomic_type::make();
    return _val;
  }
  int read() {
    auto scope = atomic_type::make();
    _val = analogRead(_pin);
    return _val;
  }
};

template <typename AtomicType = Atomic<>>
class SDC_SPI: Controller, AtomicType { // MicroSD card
public:
  using type        = SDC_SPI;
  using atomic_type = AtomicType;

protected:
  static pin_type const _pin_sdi = 19U;
  static pin_type const _pin_sdo = 23U;
  static pin_type const _pin_sck = 18U;
  static pin_type const _pin_sel = 5U;

public:
  bool init() override {
    update(msecu32_t{0});
    return true;
  }
  void update(msecu32_t const now) override {
  }
};

using tpc_type = TPC_LCD<>;
using rgb_type = RGB_PWM<>;
using amp_type = AMP_PWM<>;
using cds_type = CDS_ADC<>;
using sdc_type = SDC_SPI<>;

class ESP323248S035 : Controller {};

template <class V> class ESP323248S035C : public ESP323248S035 {
  // You must be careful to not make any lvgl API calls from any constructor,
  // since that library is initialized indirectly via method init(), and the
  // user can only call init() on a fully constructed object.
  // UI initialization and layout should be performed in the init(lv_obj_t *)
  // method of template parameter V, which we verify here derives from View.
  static_assert(std::is_base_of<View, V>::value,
                "Template parameter V must derive from View.");

protected:
  static constexpr msecu32_t _refresh = msecu32_t{20};

  // All hardware peripherals found on-board, external to the ESP32, that have
  // class drivers implemented in this library.
  // This is a singleton class, with a single instance named _hw, which is
  // instantiated in the outer/parent class's member initializer list.
  struct hw_t {
    tpc_type tpc;
    rgb_type rgb;
    amp_type amp;
    cds_type cds;
    sdc_type sdc;
    hw_t(V &view) : tpc(view) {}
  } _hw;

public:
  static inline constexpr msecu32_t refresh() { return _refresh; }

  // Accessor for members of the hardware peripherals (hw_t) singleton instance.
  //
  // Instead of using separate methods for each member, we use a template and
  // the SFINAE pattern to select the correct member based on class type of the
  // template argument.
  //
  // Note that _hw must be resolved at runtime in the body of a method, not at
  // compile time in a method's signature (e.g., as default argument).
  template <class C> inline C &hw() {
    union {
      tpc_type &cast(hw_t &p, tpc_type *) { return p.tpc; }
      rgb_type &cast(hw_t &p, rgb_type *) { return p.rgb; }
      amp_type &cast(hw_t &p, amp_type *) { return p.amp; }
      cds_type &cast(hw_t &p, cds_type *) { return p.cds; }
      sdc_type &cast(hw_t &p, sdc_type *) { return p.sdc; }
    } u;
    return u.cast(_hw, static_cast<C *>(0));
  }

public:
  ESP323248S035C() = delete;
  explicit ESP323248S035C(V &view) : _hw(view) {}

  virtual ~ESP323248S035C() {}

  bool init() {
    // Due to short-circuiting, if any member's init method returns false, the
    // remaining member init methods are never called.
    // So be sure the ordering is correct.
    return _hw.rgb.init() && _hw.amp.init() && _hw.cds.init() &&
           _hw.tpc.init() && _hw.sdc.init();
  }

  virtual inline void update(msecu32_t const = msecu32()) override {
    dispatch(&_hw.sdc, &_hw.cds, &_hw.rgb, &_hw.tpc, &_hw.amp);
  }

  // This will call msecu32() on each peripheral in the order they are listed
  // so that the runtime of each peripheral can be calculated correctly:
  //    |<-- e[0] -->|<-- e[1] -->| ... |<-- e[n] -->|
  //    ^t[0]        ^t[1]              ^t[n]
  //
  // Otherwise, if we were to call msecu32() only once at the beginning, the
  // update frequency of the peripherals would be skewed:
  //
  //   |<-- e[0] -->|
  //   |<--       e[1]        -->|
  //     ...
  //   |<--                   e[n]               -->|
  template <class... E> void dispatch(E... e) {
    (int[]){(e->update(msecu32()), 0)...};
  }
};

} // namespace bsp
