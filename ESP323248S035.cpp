#include "ESP323248S035.h"

#include "debug.h"

// THROTTLE invokes <fn> if and only if <per> milliseconds have elapsed since
// the last time <fn> was invoked from this macro (or if it is being invoked for
// the first time).
//
// This is a simple (naïve) way to implement periodic function calls without
// blocking (no delay()) on a single thread. If insufficient time has elapsed,
// nothing is invoked and execution continues.
//
// THROTTLE is not interrupt-driven or atomic. It will gladly run <fn> forever
// if permitted.
//
// Call THROTTLE as frequently as possible for best accuracy.
//
// Concatenate <per> to "prev_" for the static variable name so that multiple
// calls to THROTTLE can be nested in a common scope.
// syntax of <per> must be a variable identifier, macro, or numeric literal.
#define THROTTLE(fn,now,per) {         \
    static msec_t prev_ ## per = 0UL; \
    if ((now) - prev_ ## per >= per) { \
      fn;                              \
      prev_ ## per = (now);            \
    }                                  \
  }

lv_disp_t          *TPC_LCD::_disp;
lv_disp_drv_t       TPC_LCD::_ddrv;
lv_disp_draw_buf_t  TPC_LCD::_draw;
lv_color_t          TPC_LCD::_fb[TPC_LCD::_tft_width * 10];
lv_indev_t         *TPC_LCD::_inpt;
lv_indev_drv_t      TPC_LCD::_idrv;

ESP323248S035C hmi;

#if (LV_USE_LOG)
static void c_log(const char *buf) { swritef(Serial, "%s", buf); }
#endif

void c_tick() { lv_tick_inc(TPC_LCD::refresh()); }

void c_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color) {
  hmi.lcd.flush(drv, area, color);
}

void c_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  hmi.lcd.read(drv, data);
}

bool TPC_LCD::init() {
#if (LV_USE_LOG)
  lv_log_register_print_cb(c_log);
#endif

  lv_init();
  SPIClass::begin(_pin_sck, _pin_sdi, _pin_sdo);


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
  tx(cmd_t::colmod, _tft_depth); // 16 bits R5G6B5
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

  lv_disp_draw_buf_init(&_draw, _fb, NULL, _tft_width * 10);
  lv_disp_drv_init(&_ddrv);
  _ddrv.hor_res = width();
  _ddrv.ver_res = height();
  _ddrv.flush_cb = c_flush;
  _ddrv.draw_buf = &_draw;
  _disp = lv_disp_drv_register(&_ddrv);

  lv_obj_clean(lv_scr_act());

  // TODO: calibrate touch?
  bool ok = false;
  if (TwoWire::begin(_pin_sda, _pin_scl)) {
    static uint8_t pid[_size_prod_id];
    if ((ok = rx(_reg_prod_id, pid, sizeof(pid)))) {
      swritef(Serial, "TPC_I2C: device ID = %u");
    }
  }

  lv_indev_drv_init(&_idrv);
  _idrv.type = LV_INDEV_TYPE_POINTER;
  _idrv.read_cb = c_read;
  _inpt = lv_indev_drv_register(&_idrv);

  _tick.attach_ms(_tft_refresh, c_tick);

  return ok;
}

void TPC_LCD::flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color) {
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
  px(cmd_t::ramwr, color, lv_area_get_width(area) * lv_area_get_height(area));
  lv_disp_flush_ready(drv);
}

void TPC_LCD::read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  static int16_t _x = 0, _y = 0;
  point_t pt;
  switch (touch_count()) {
  case 1:
    if (map(&pt, 1)) {
      swritef(Serial, "touch: (%d,%d)", pt.x, pt.y);
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

bool RGB_PWM::init() {
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
  return true;
}

void RGB_PWM::set(lv_color32_t const rgb) {
  ledcWrite(_pwm_r_chan, _pwm_hres - rgb.ch.red);
  ledcWrite(_pwm_g_chan, _pwm_hres - rgb.ch.green);
  ledcWrite(_pwm_b_chan, _pwm_hres - rgb.ch.blue);
}

bool AMP_PWM::init() {
  pinMode(_pin, INPUT); // High impedence
  return true;
}

void AMP_PWM::set(uint32_t const hz, msec_t const ms) {
  tone(_pin, hz, ms);
}

void AMP_PWM::mute() {
  noTone(_pin);
}

bool CDS_ADC::init() {
  analogSetAttenuation(_att); // 0dB(1.0x) 0~800mV
  pinMode(_pin, INPUT);
  return true;
}

int CDS_ADC::get() {
  return analogRead(_pin);
}

bool SDC_SPI::init() {
  return true;
}

ESP323248S035C::ESP323248S035C() {}
ESP323248S035C::~ESP323248S035C() {}

bool ESP323248S035C::init() {
  // Due to short-circuiting, if any member's init method returns false, the
  // remaining member init methods are never called.
  // So be sure the ordering is correct.
  return
    rgb.init() &&
    amp.init() &&
    cds.init() &&
    lcd.init() &&
    sdc.init() ;
}

void ESP323248S035C::update(msec_t const now) {

  lv_timer_handler();
  //THROTTLE(, now, _refresh);
}
