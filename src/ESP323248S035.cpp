#include "ESP323248S035.h"

#include "callback.hpp"

// THROTTLE invokes <fn> if and only if <per> milliseconds have elapsed since
// the last time <fn> was invoked from this macro (or if it is being invoked for
// the first time).
//
// This is a simple naïve way to implement periodic function calls on a single
// thread without blocking (no delay()). If insufficient time has elapsed,
// nothing is invoked and execution continues.
//
// THROTTLE is not interrupt-driven or atomic. It will gladly run <fn> forever
// or never at all under the right circumstances. Since it runs on the main
// thread, it's reasonable to expect to be preempted at any moment.
//
// Call THROTTLE as frequently as possible for best accuracy.
//
// Concatenate <per> to "prev_" for the static variable name so that multiple
// calls to THROTTLE can be nested in a common scope.
// Syntax of <per> must be a variable identifier, macro, or numeric literal —
// notably, this excludes expressions like arithmetic and application "()".
#define THROTTLE(fn,now,per) {         \
    static msec_t prev_ ## per = 0UL;  \
    if ((now) - prev_ ## per >= per) { \
      fn;                              \
      prev_ ## per = (now);            \
    }                                  \
  }

// lvgl C API functions
#if (LV_USE_LOG)
typedef void (*log_t)(const char *);
#endif
typedef void (*tick_t)();
typedef void (*flush_t)(lv_disp_drv_t *, const lv_area_t *, lv_color_t *);
typedef void (*read_t)(lv_indev_drv_t *, lv_indev_data_t *);
typedef void (*scroll_begin_t)(lv_event_t *);

lv_disp_t          *TPC_LCD::_disp;
lv_disp_drv_t       TPC_LCD::_ddrv;
lv_disp_draw_buf_t  TPC_LCD::_draw;
lv_color_t          TPC_LCD::_fb[TPC_LCD::_tft_width * 10];
lv_indev_t         *TPC_LCD::_inpt;
lv_indev_drv_t      TPC_LCD::_idrv;

bool TPC_LCD::init() {
#if (LV_USE_LOG)
  lv_log_register_print_cb(static_cast<log_t>(&TPC_LCD::log));
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
  Callback<void(lv_disp_drv_t *, const lv_area_t *, lv_color_t *)>::fn =
    std::bind(&TPC_LCD::flush, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  _ddrv.flush_cb = static_cast<flush_t>(
    Callback<void(lv_disp_drv_t *, const lv_area_t *, lv_color_t *)>::callback);
  _ddrv.draw_buf = &_draw;
  _disp = lv_disp_drv_register(&_ddrv);

  lv_obj_clean(lv_scr_act());

  // TODO: calibrate touch?
  bool ok;

  if ((ok = TwoWire::begin(_pin_sda, _pin_scl))) {
    static uint8_t pid[_size_prod_id];
    if ((ok = rx(_reg_prod_id, pid, sizeof(pid)))) {
      swritef(Serial, "TPC_I2C: device ID = %u");
    }
  }

  lv_indev_drv_init(&_idrv);
  _idrv.type = LV_INDEV_TYPE_POINTER;
  Callback<void(lv_indev_drv_t *, lv_indev_data_t *)>::fn =
    std::bind(&TPC_LCD::read, this,
    std::placeholders::_1, std::placeholders::_2);
  _idrv.read_cb = static_cast<read_t>(
    Callback<void(lv_indev_drv_t *, lv_indev_data_t *)>::callback);
  _inpt = lv_indev_drv_register(&_idrv);

  _tick.attach_ms(_tft_refresh, static_cast<tick_t>(&TPC_LCD::tick));

  layout();

  return ok;
}

void TPC_LCD::update(msec_t const now) {
  lv_timer_handler();
}

void TPC_LCD::layout() {

  /*Create a Tab view object*/
  lv_obj_t *tabview;
  tabview = lv_tabview_create(lv_scr_act(), LV_DIR_LEFT, 80);

  lv_obj_add_event_cb(
    lv_tabview_get_content(tabview),
    static_cast<scroll_begin_t>(&TPC_LCD::scroll_begin),
    LV_EVENT_SCROLL_BEGIN, NULL);

  lv_obj_set_style_bg_color(tabview, lv_palette_lighten(LV_PALETTE_RED, 2), 0);

  lv_obj_t * tab_btns = lv_tabview_get_tab_btns(tabview);
  lv_obj_set_style_bg_color(tab_btns, lv_palette_darken(LV_PALETTE_GREY, 3), 0);
  lv_obj_set_style_text_color(tab_btns, lv_palette_lighten(LV_PALETTE_GREY, 5), 0);
  lv_obj_set_style_border_side(tab_btns, LV_BORDER_SIDE_RIGHT, LV_PART_ITEMS | LV_STATE_CHECKED);


  /*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
  lv_obj_t *tab1 = lv_tabview_add_tab(tabview, "Tab 1");
  lv_obj_t *tab2 = lv_tabview_add_tab(tabview, "Tab 2");
  lv_obj_t *tab3 = lv_tabview_add_tab(tabview, "Tab 3");
  lv_obj_t *tab4 = lv_tabview_add_tab(tabview, "Tab 4");
  lv_obj_t *tab5 = lv_tabview_add_tab(tabview, "Tab 5");

  lv_obj_set_style_bg_color(tab2, lv_palette_lighten(LV_PALETTE_AMBER, 3), 0);
  lv_obj_set_style_bg_opa(tab2, LV_OPA_COVER, 0);

  /*Add content to the tabs*/
  //    lv_obj_t * label = lv_label_create(tab1);
  //    lv_label_set_text(label, "First tab");
  lv_obj_t                  *_scaleChart;
  // lv_chart_series_t         *_scaleSeries;

  _scaleChart = lv_chart_create(tab1);
  lv_obj_set_size(_scaleChart, _tft_width - 80 - 10, _tft_height - 10);
  lv_obj_center(_scaleChart);
  lv_chart_set_type(_scaleChart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/

  /*Add two data series*/
  lv_chart_add_series(_scaleChart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_point_count(_scaleChart, 100);
  lv_chart_set_update_mode(_scaleChart, LV_CHART_UPDATE_MODE_SHIFT);


  lv_obj_t *label = lv_label_create(tab2);
  // lv_obj_set_style_text_font(label, &beans_16, 0);
  lv_label_set_text(label, "Second tab");

  label = lv_label_create(tab3);
  lv_label_set_text(label, "Third tab");

  label = lv_label_create(tab4);
  lv_label_set_text(label, "Forth tab");

  label = lv_label_create(tab5);
  lv_label_set_text(label, "Fifth tab");

  lv_obj_clear_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);

}

inline void TPC_LCD::scroll_begin(lv_event_t * e)
{
  /*Disable the scroll animations. Triggered when a tab button is clicked */
  if (lv_event_get_code(e) == LV_EVENT_SCROLL_BEGIN) {
    lv_anim_t *a = (lv_anim_t *)lv_event_get_param(e);
    if (a) { a->time = 0; }
  }
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

  set(lv_color32_t{.full = 0x003300AA});

  return true;
}

void RGB_PWM::update(msec_t const now) {
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

void AMP_PWM::update(msec_t const now) {
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

void CDS_ADC::update(msec_t const now) {
}

int CDS_ADC::get() {
  return analogRead(_pin);
}

bool SDC_SPI::init() {
  return true;
}

void SDC_SPI::update(msec_t const now) {
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
  THROTTLE(update(now, &sdc, &cds, &rgb, &lcd, &amp), now, _refresh);
}
