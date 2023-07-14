#ifndef ESP323248S035_h
#define ESP323248S035_h

#include <Arduino.h>
#include <Ticker.h>
#include <SPI.h>
#include <Wire.h>
#include <lvgl.h>
#include <StatusLED.h>

#include <stdint.h>
#include <chrono>
#include <utility>
#include <mutex>

#include "debug.hpp"

// The standard unit of time used herein is msec_t (milliseconds).
// Use method count() to obtain the value in its underlying type (uint32_t).
typedef std::chrono::duration<uint32_t, std::milli> msec_t;
inline msec_t msecs() { return msec_t{millis()}; }

// The sole template parameter msec_t N defines the frequency (miilliseconds)
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
//       the watchdog timer to reset the system. This will ensure func runs as
//       frequently as possible, but it might cause system resets.
//     NotDefined(TASK_SCHED_GREEDY):
//       The subsequent func is deferred to the following period.
//       This means the func whose deadline was missed will never execute, which
//       reduces the frequency of func but gives the IDLE task more opportunity
//       to run and reset the watchdog timer.
//       This behavior should improve system stability.
//       Therefore, it is the default.
//
template <const uint32_t N>
void govern(std::function<void(void)> func) {
  // xTaskDelayUntil arguments are expressed in units of system "ticks".
  // Conventionally, 1 tick = 1 millisecond regardless of CPU frequency.
  static constexpr TickType_t freq = N / portTICK_PERIOD_MS;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
#ifdef TASK_SCHED_GREEDY
    // vTaskDelayUntil is an older function that does not return a value.
    // It was replaced with xTaskDelayUntil but still exists for compat.
    // This function does not provide a way to determine if the task was
    // actually delayed or not.
    vTaskDelayUntil(&last, freq);
#else
    if (xTaskDelayUntil(&last, freq))
#endif
    { func(); }
  }
}

typedef int8_t gpin_t;

struct Cyclic {
  virtual void update(msec_t const) = 0;
};

struct Periodic: Cyclic {
  virtual bool init() = 0;
};

struct View: Cyclic {
  virtual bool init(lv_obj_t *) = 0;
  virtual std::string title() = 0;
};

template <typename T>
struct Viewable {
    constexpr static bool check(View *) { return true; }
    constexpr static bool check(...)    { return false; }
    enum { value = check(static_cast<T*>(0)) };
};

class TPC_LCD: public Periodic, public SPIClass, public TwoWire {
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
  enum class cmd_t: uint8_t {
    swreset = 0x01, // Software Reset
    slpin   = 0x10, // Sleep in
    slpout  = 0x11, // Sleep out
    noron   = 0x13, // Normal Display Mode On
    invoff  = 0x20, // Display Inversion Off
    dispon  = 0x29, // Display On
    caset   = 0x2A, // Column Address Set
    raset   = 0x2B, // Row Address Set
    ramwr   = 0x2C, // Memory Write
    madctl  = 0x36, // Memory Data Access Control
    colmod  = 0x3A, // Interface Pixel Format
    pgc     = 0xE0, // Positive Gamma Control
    ngc     = 0xE1, // Negative Gamma Control
    cscon   = 0xF0, // Command Set wqaControl
  };
  enum class madctl_t: uint8_t {
    my      = 0x80, // Row Addr Order: 0=Inc(↓), 1=Dec(↑)
    mx      = 0x40, // Column Addr Order: 0=Inc(→), 1=Dec(←)
    mv      = 0x20, // Row/Col exchange: 0=Norm, 1=Exchange
    ml      = 0x10, // Vertical Refresh Order
    bgr     = 0x08, // Subpixel rendering: BGR order
    mh      = 0x10, // Horizontal Refresh Order
    rgb     = 0x00, // Subpixel rendering: RGB order
    // MADCTL rotation bitmasks
    // We can conveniently translate these bitmasks to unique values 0..3, andddd
    // preserve the ordering for computing degrees of rotation, as follows:
    //    enum: ((mask >> 5) & 3)          degrees: enum * 90
    tall          = my,           // ((128 >> 5) & 3) * 90 == 0 * 90 =   0°
    wide          = mv,           //  ((32 >> 5) & 3) * 90 == 1 * 90 =  90°
    tall_inverted = mx,           //  ((64 >> 5) & 3) * 90 == 2 * 90 = 180°
    wide_inverted = my | mv | mx, // ((224 >> 5) & 3) * 90 == 3 * 90 = 270°
  };
  enum class colmod_t: uint8_t {
    rgb16   = 0x50,
    ctrl16  = 0x05,
    rgb656  = rgb16 | ctrl16,
  };

  // GPIO pins
  static gpin_t   constexpr _pin_sdi        = 12U; // SPI MISO
  static gpin_t   constexpr _pin_sdo        = 13U; // SPI MOSI
  static gpin_t   constexpr _pin_sck        = 14U; // SPI SCLK
  static gpin_t   constexpr _pin_sel        = 15U; // SPI SS/CS
  static gpin_t   constexpr _pin_sdc        =  2U; // LCD DC/RS
  static gpin_t   constexpr _pin_blt        = 27U; // delicious BLT
  static gpin_t   constexpr _pin_scl        = 32U; // I²C SCL
  static gpin_t   constexpr _pin_sda        = 33U; // I²C SDA
  static gpin_t   constexpr _pin_int        = 21U; // TPC INT
  static gpin_t   constexpr _pin_rst        = 25U; // TPC RST
  // SPI interfaces
  static uint8_t  constexpr _spi_bus        = 2U;   // HSPI
  static uint32_t constexpr _bus_freq       = 80000000U; // 80 MHz
  static uint8_t  constexpr _word_ord       = SPI_MSBFIRST;
  static uint8_t  constexpr _sig_mode       = SPI_MODE0;
  // I²C interfaces
  static uint8_t  constexpr _i2c_bus        = 1U;   // Wire1
  static uint8_t  constexpr _dev_addr       = 0x5D; // I²C touch device address
  // Backlight PWM
  static uint8_t  constexpr _pwm_blt_chan   = 12U;
  static uint32_t constexpr _pwm_blt_freq   = 5000U; // 5 kHz
  static uint8_t  constexpr _pwm_blt_bits   = 8U;
  static uint8_t  constexpr _pwm_blt_hres   = (1U << _pwm_blt_bits) - 1U;
  // TFT configuration
  static msec_t   constexpr _tft_refresh    = msec_t{10};  // milliseconds
  static uint16_t constexpr _tft_width      = 320U; // Physical dimensions, does
  static uint16_t constexpr _tft_height     = 480U; // not consider rotations.
  static uint8_t  constexpr _tft_depth      = static_cast<uint8_t>(colmod_t::rgb656);
  static uint8_t  constexpr _tft_subpixel   = static_cast<uint8_t>(madctl_t::rgb);
  static uint8_t  constexpr _tft_aspect     = static_cast<uint8_t>(madctl_t::wide_inverted);
  // Touch MMIO
  static uint16_t constexpr _reg_prod_id    = 0x8140; // product ID (byte 1/4)
  static size_t   constexpr _size_prod_id   = 4U;     // bytes
  static uint16_t constexpr _reg_stat       = 0x814E; // buffer/track status
  static uint16_t constexpr _reg_base_point = 0x814F;
  static size_t   constexpr _touch_max      = 5U; // simultaneous touch points

  // lvgl API methods
#if (LV_USE_LOG)
  static inline void log(const char *buf) { writef(Serial, "%s", buf); }
#endif
  static inline void tick() { lv_tick_inc(_tft_refresh.count()); }
  void flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color);
  void read(lv_indev_drv_t *drv, lv_indev_data_t *data);

  template <class ...E>
  void tx(cmd_t const code, E... e) {
    constexpr size_t size = sizeof...(e);
    uint8_t data[size] = { static_cast<uint8_t>(e)... };
    digitalWrite(_pin_sdc, LOW); // D/C ⇒ command
    beginTransaction(SPISettings(_bus_freq, _word_ord, _sig_mode));
    digitalWrite(_pin_sel, LOW); // C/S ⇒ enable
    SPIClass::write(static_cast<uint8_t>(code));
    if (size > 0) {
      digitalWrite(_pin_sdc, HIGH); // D/C ⇒ data
      writeBytes(data, size);
    }
    digitalWrite(_pin_sel, HIGH); // C/S ⇒ disable
    endTransaction();
  }

  void px(cmd_t const code, const lv_color_t color[], size_t const count) {
    digitalWrite(_pin_sdc, LOW); // D/C ⇒ command
    beginTransaction(SPISettings(_bus_freq, _word_ord, _sig_mode));
    digitalWrite(_pin_sel, LOW); // C/S ⇒ enable
    SPIClass::write(static_cast<uint8_t>(code));
    if (count > 0) {
      digitalWrite(_pin_sdc, HIGH); // D/C ⇒ data
      writePixels(color, count * sizeof(lv_color_t));
    }
    digitalWrite(_pin_sel, HIGH); // C/S ⇒ disable
    endTransaction();
  }

  inline bool r16(uint16_t const reg) {
    beginTransmission(_dev_addr);
    return TwoWire::write(static_cast<uint8_t>(reg >> 8)) && // MSB (first)
      TwoWire::write(static_cast<uint8_t>(reg & 0xFF));      // LSB
  }

  inline bool tx(uint16_t const reg, uint8_t * const buf, size_t const len) {
    size_t sent = 0U;
    if (r16(reg)) {
      sent = TwoWire::write(buf, len);
    }
    endTransmission();
    return sent == len;
  }

  inline bool rx(uint16_t const reg, uint8_t * const buf, size_t const len) {
    uint8_t *dst = buf;
    size_t rem = 0U;
    bool ok = r16(reg);
    endTransmission(false);
    if (ok && (len == (rem = requestFrom(_dev_addr, len)))) {
      while (available() && rem--) {
        *(dst++) = TwoWire::read();
      }
    }
    return rem == 0 && dst != buf;
  }

  inline bool rx(uint16_t const reg, point_t * const buf, size_t const len) {
    return rx(reg, (uint8_t *)buf, len);
  }

  // template <unsigned N = 0>
  // static inline constexpr uint16_t track() {
  //   return _reg_base_point + N * sizeof(point_t) + offsetof(point_t, track);
  // }

  template<uint16_t R = _tft_aspect>
  inline bool map(point_t * const pt, size_t count) {
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
      }
    }[(R>>5) & 3])();
    return ok;
  }

  inline size_t touch_count() {
    static uint8_t stat[1] = { 0 };
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
  static lv_disp_t          *_disp;
  static lv_disp_drv_t       _ddrv;
  static lv_disp_draw_buf_t  _draw;
  static lv_color_t          _fb[_tft_width * 10];
  static lv_indev_t         *_inpt;
  static lv_indev_drv_t      _idrv;

  std::mutex _mutx;

  Ticker _tick;
  View &_view;

public:
  TPC_LCD() = delete; // display driver must have a root view!
  explicit TPC_LCD(View &root):
    SPIClass(_spi_bus), TwoWire(_i2c_bus), _view(root) {}
  virtual ~TPC_LCD() {}

  bool init() override;
  void update(msec_t const now) override;

  template<uint8_t C = _pwm_blt_chan>
  static inline void set_backlight(uint16_t duty) { ledcWrite(C, duty); }

  static inline constexpr bool wide() {
    return (_tft_aspect & static_cast<uint8_t>(madctl_t::mv)) > 0; // transpose?
  }
  static inline constexpr uint16_t width() {
    return wide() ? _tft_height : _tft_width;
  }
  static inline constexpr uint16_t height() {
    return wide() ? _tft_width : _tft_height;
  }
};

class RGB_PWM: Periodic, public StatusLED { // RGB 3-pin analog LED
private:
protected:
  static gpin_t const _pin_r =  4U;
  static gpin_t const _pin_g = 16U;
  static gpin_t const _pin_b = 17U;

  static uint8_t  const _pwm_r_chan = 13U;
  static uint8_t  const _pwm_g_chan = 14U;
  static uint8_t  const _pwm_b_chan = 15U;

  static uint32_t const _pwm_freq   = 5000U; // 5 kHz
  static uint8_t  const _pwm_bits   = 8U;
  static uint8_t  const _pwm_hres   = (1U << _pwm_bits) - 1U;

private:
  std::mutex _mutx;

public:
  RGB_PWM():
    StatusLED(
      _pin_r, _pin_g, _pin_b, HIGH, StatusLEDMode::Fixed, 100U, true,
        COLOR_RED, 0x3F
    ){
      setWrite([&](SRGB const &rgb) {
        // write() always clips its actual argument's components to 8-bits,
        // so there is no narrowing happening in these typecasts.
        set(LV_COLOR_MAKE32(
          (uint8_t)rgb.red, (uint8_t)rgb.green, (uint8_t)rgb.blue));
      });
    }
  bool init() override;
  void update(msec_t const now) override;

  void set(lv_color32_t const rgb);
};

class AMP_PWM: Periodic { // Autio amplifier
private:
  std::mutex _mutx;

protected:
  static gpin_t const _pin = 26U;

public:
  bool init() override;
  void update(msec_t const now) override;

  void set(uint32_t const hz, msec_t const ms);
  void mute();
};

class CDS_ADC: Periodic { // Light sensitive photo-resistor
private:
  std::mutex _mutx;

protected:
  static adc_attenuation_t const _att = ADC_0db; // 0dB (1.0x): 0~800mV

  static gpin_t const _pin = 34U; // A0

public:
  bool init() override;
  void update(msec_t const now) override;

  int get();
};

class SDC_SPI: Periodic { // MicroSD card
private:
  std::mutex _mutx;

protected:
  static gpin_t const _pin_sdi = 19U;
  static gpin_t const _pin_sdo = 23U;
  static gpin_t const _pin_sck = 18U;
  static gpin_t const _pin_sel =  5U;

public:
  bool init() override;
  void update(msec_t const now) override;
};

class ESP323248S035: Periodic {};

template <class V>
class ESP323248S035C: public ESP323248S035 {
  // You must be careful to not make any lvgl API calls from C's default
  // constructor, since the library will be initialized after C::C() is called.
  // Initialization should be performed in View::init(lv_obj_t *).
  static_assert(Viewable<V>::value);

protected:
  static constexpr msec_t _refresh = msec_t{1};
public:
  static inline constexpr msec_t refresh() { return _refresh; }

protected:
  TPC_LCD _tpc;
  RGB_PWM _rgb;
  AMP_PWM _amp;
  CDS_ADC _cds;
  SDC_SPI _sdc;

public:
  ESP323248S035C(V &view): _tpc(view) {}
  // ESP323248S035C(V &&view):

  virtual ~ESP323248S035C() {}

  // We must call msecs() for each peripheral so that the time spent servicing
  // the first peripherals doesn't influence the update frequency of the last.
  template <class ...E>
  void dispatch(E... e) { (int[]){ (e->update(msecs()), 0)... }; }

  bool init() {
    // Due to short-circuiting, if any member's init method returns false, the
    // remaining member init methods are never called.
    // So be sure the ordering is correct.
    return
      _rgb.init() &&
      _amp.init() &&
      _cds.init() &&
      _tpc.init() &&
      _sdc.init() ;
  }

  void update(msec_t const now = msecs()) {
    dispatch(&_sdc, &_cds, &_rgb, &_tpc, &_amp);
  }

  inline TPC_LCD & tpc() { return _tpc; }
  inline RGB_PWM & rgb() { return _rgb; }
  inline AMP_PWM & amp() { return _amp; }
  inline CDS_ADC & cds() { return _cds; }
  inline SDC_SPI & sdc() { return _sdc; }
};

#endif // ESP323248S035_h
