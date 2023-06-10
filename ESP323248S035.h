#ifndef ESP323248S035_h
#define ESP323248S035_h

#include <Arduino.h>
#include <Ticker.h>
#include <SPI.h>
#include <Wire.h>
#include <lvgl.h>

typedef unsigned long msec_t;
typedef int8_t        gpin_t;

template<typename... Args>
size_t swritef(Stream &out, const char * fmt, Args... args) {
  static constexpr char eol[] = "\r\n";
  // Cannot rely on the target implementation of Stream to provide printf
  size_t size = snprintf(nullptr, 0, fmt, args...);
  std::string str;
  str.reserve(size + 1);
  str.resize(size);
  snprintf(&str[0], size + 1, fmt, args...);
  return out.write(str.c_str(), size) + out.write(eol);
}

class TFT_SPI: public SPIClass { // ST7796 3.5" 320x480 TFT LCD
protected:
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
  };
  enum class colmod_t: uint8_t {
    rgb16   = 0x50,
    ctrl16  = 0x05,
    rgb656  = rgb16 | ctrl16,
  };

public:
  class rotate_t {
  public:
    enum orientation_t: int {
      invalid = 0,
      portrait,
      landscape,
      portrait_flip,
      landscape_flip,
      count,
    };
    rotate_t() = default;
    constexpr rotate_t(int d): _orientation(
      d == _int[portrait]       ? portrait : (
      d == _int[landscape]      ? landscape : (
      d == _int[portrait_flip]  ? portrait_flip : (
      d == _int[landscape_flip] ? landscape_flip : (
      invalid))))
    ) {}
    constexpr rotate_t(orientation_t o): _orientation(o) {}
    constexpr operator orientation_t() const { return _orientation; }
    constexpr operator bool() const {
      return invalid < _orientation && _orientation < count;
    }
    constexpr operator int() const {
      return *this ? _int[_orientation] : _int[invalid];
    }
    constexpr operator madctl_t() const {
      return *this ? _madctl_t[_orientation] : _madctl_t[invalid];
    }
    constexpr bool wide_aspect() { return (int)*this % 180 != 0; }
  private:
    // rotation angle (degrees)
    static constexpr int _int[] = { 360, 0, 90, 180, 270, };
    // MADCTL bitmasks for screen rotation
    static constexpr madctl_t _madctl_t[] = {
      static_cast<madctl_t>(0x00), // invalid
      madctl_t::my,                // portrait
      madctl_t::mv,                // landscape
      madctl_t::mx,                // portrait_flip
      static_cast<madctl_t>(       // landscape_flip
        static_cast<uint8_t>(madctl_t::my) |
        static_cast<uint8_t>(madctl_t::mv) |
        static_cast<uint8_t>(madctl_t::mx)),
    };
    orientation_t _orientation;
  };

protected:
  // GPIO pins
  static gpin_t const _pin_sdi = 12U; // MISO
  static gpin_t const _pin_sdo = 13U; // MOSI
  static gpin_t const _pin_sck = 14U; // SCLK
  static gpin_t const _pin_sel = 15U; // SS/CS
  static gpin_t const _pin_sdc =  2U; // DC/RS
  static gpin_t const _pin_blt = 27U; // delicious

  // SPI line discipline
  static uint32_t const _bus_freq = 80000000U; // 80 MHz
  static uint8_t  const _word_ord = SPI_MSBFIRST;
  static uint8_t  const _sig_mode = SPI_MODE0;

  // Backlight PWM
  static uint8_t  const _pwm_blt_chan = 12U;
  static uint32_t const _pwm_blt_freq = 5000U; // 5 kHz
  static uint8_t  const _pwm_blt_bits = 8U;
  static uint8_t  const _pwm_blt_hres = (1U << _pwm_blt_bits) - 1U;

  // TFT configuration
  static msec_t   const _tft_refresh =  10U;
  static uint16_t const _tft_width   = 320U; // Use methods width() and height()
  static uint16_t const _tft_height  = 480U; //  to account for rotation.

  static uint8_t  const _tft_colors = (uint8_t)colmod_t::rgb656;
  static uint8_t  const _tft_subord = (uint8_t)madctl_t::rgb;
  static int      const _tft_rotate = 90; // landscape

  template <class ...E>
  void command(cmd_t const code, E ...e) {
    constexpr size_t size = sizeof...(e);
    uint8_t data[size] = { static_cast<uint8_t>(e)... };
    digitalWrite(_pin_sdc, LOW); // D/C ⇒ command
    beginTransaction(SPISettings(_bus_freq, _word_ord, _sig_mode));
    digitalWrite(_pin_sel, LOW); // C/S ⇒ enable
    write((uint8_t)code);
    if (size > 0) {
      digitalWrite(_pin_sdc, HIGH); // D/C ⇒ data
      writeBytes(data, size);
    }
    digitalWrite(_pin_sel, HIGH); // C/S ⇒ disable
    endTransaction();
  }

  void pixels(cmd_t const code, const lv_color_t color[], size_t const count);

private:
  static lv_disp_t          *_disp;
  static lv_disp_drv_t       _ddrv;
  static lv_disp_draw_buf_t  _draw;
  static lv_color_t          _fb[_tft_width * 10];
  static lv_indev_t         *_inpt;
  static lv_indev_drv_t      _idrv;

  Ticker _tick;

public:
  TFT_SPI() = default;

  bool init();
  void flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color);

  inline void set_backlight(uint16_t duty) { ledcWrite(_pwm_blt_chan, duty); }

  static inline constexpr uint8_t pixel_order() { return _tft_subord; }
  static inline constexpr msec_t refresh() { return _tft_refresh; }
  static inline constexpr uint16_t width() {
    return rotate_t(_tft_rotate).wide_aspect() ? _tft_height : _tft_width;
  }
  static inline constexpr uint16_t height() {
    return rotate_t(_tft_rotate).wide_aspect() ? _tft_width : _tft_height;
  }
  static inline constexpr rotate_t rotation() {
    return _tft_rotate;
  }
};

class TPC_I2C: public TwoWire { // GT911 capacitive touch
private:
protected:
  struct __attribute__((packed)) point {
    // 0x814F-0x8156, ... 0x8176 (5 points)
    uint8_t track;
    uint16_t x; // unaligned
    uint16_t y;
    uint16_t area;
    uint8_t _u8[1];
  };

  static uint8_t const _bus_id   = 1U; // extern TwoWire Wire1

  static uint8_t  const _dev_addr = 0x5D;   // I²C device address
  static uint16_t const _reg_pid1 = 0x8140; // product ID (byte 1/4)
  static uint16_t const _reg_stat = 0x814E; // buffer/track status
  static uint16_t const _reg_tid1 = 0x814F; // track ID

  static size_t const _size_pid = 4U; // bytes

  static gpin_t const _pin_scl = 32U;
  static gpin_t const _pin_sda = 33U;
  static gpin_t const _pin_int = 21U;
  static gpin_t const _pin_rst = 25U;

  static size_t const _touch_max = 5U; // simultaneous touch points

  inline bool reg16(uint16_t const reg) {
    beginTransmission(_dev_addr);
    return TwoWire::write(static_cast<uint8_t>(reg >> 8)) && // MSB (first)
      TwoWire::write(static_cast<uint8_t>(reg & 0xFF));      // LSB
  }
  inline bool write(uint16_t const reg, uint8_t * const buf, size_t const len) {
    size_t sent = 0U;
    if (reg16(reg)) {
      sent = TwoWire::write(buf, len);
    }
    endTransmission();
    return sent == len;
  }
  inline bool read(uint16_t const reg, uint8_t * const buf, size_t const len) {
    uint8_t *dst = buf;
    size_t rem = 0U;
    bool ok = reg16(reg);
    endTransmission(false);
    if (ok && (len == (rem = requestFrom(_dev_addr, len)))) {
      while (available() && rem--) {
        *(dst++) = TwoWire::read();
      }
    }
    return rem == 0 && dst != buf;
  }
  inline bool read(uint16_t const reg, point * const buf, size_t const len) {
    return read(reg, (uint8_t *)buf, len);
  }

public:
  TPC_I2C(): TwoWire(_bus_id) {}
  bool init();
  size_t count();
  bool get(point * const pt, size_t count);
  void read(lv_indev_drv_t *drv, lv_indev_data_t *data);
};

class RGB_PWM { // RGB 3-pin analog LED
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

public:
  bool init();
  void set(lv_color32_t const rgb);
};

class AMP_PWM { // Autio amplifier
private:
protected:
  static gpin_t const _pin = 26U;

public:
  bool init();
  void set(uint32_t const hz, msec_t const ms);
  void mute();
};

class CDS_ADC { // Light sensitive photo-resistor
private:
protected:
  static adc_attenuation_t const _att = ADC_0db; // 0dB (1.0x): 0~800mV

  static gpin_t const _pin = 34U; // A0

public:
  bool init();
  int get();
};

class SDC_SPI { // MicroSD card
private:
protected:
  static gpin_t const _pin_sdi = 19U;
  static gpin_t const _pin_sdo = 23U;
  static gpin_t const _pin_sck = 18U;
  static gpin_t const _pin_sel =  5U;

public:
  bool init();
};

class ESP323248S035 {
private:
protected:
public:
  virtual bool init() = 0;
  virtual void update(msec_t const now) = 0;
};

class ESP323248S035C: public ESP323248S035 {
private:
  static msec_t const _refresh = 5U;
protected:
public:
  TFT_SPI tft;
  TPC_I2C tpc;
  RGB_PWM rgb;
  AMP_PWM amp;
  CDS_ADC cds;
  SDC_SPI sdc;
  ESP323248S035C();
  virtual ~ESP323248S035C();
  bool init();
  void update(msec_t const now);
};

extern ESP323248S035C hmi;

#endif // ESP323248S035_h
