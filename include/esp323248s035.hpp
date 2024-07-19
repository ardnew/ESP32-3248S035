#pragma once

// =============================================================================
//
#pragma region "Preprocessor macros"

// -----------------------------------------------------------------------------
#pragma region "Include dependencies"

// ESP-IDF SDK
#include <esp_idf_version.h>
#include <esp_check.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/portmacro.h>
// #include <esp_task_wdt.h>
#include <esp_freertos_hooks.h>
#include <esp_lcd_panel_commands.h>
#include <esp_lcd_panel_interface.h>
#include <esp_lcd_panel_io_interface.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <driver/gpio.h>
#include <rom/gpio.h>
#include <driver/ledc.h>
#include <driver/spi_master.h>
#include <driver/spi_common.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include <esp_private/spi_common_internal.h>
#else
#include <driver/spi_common_internal.h>
#endif
#include <driver/i2c.h>
#include <soc/ledc_periph.h>
// External libraries
#include <lvgl.h>
#include <drivers/display/st7796/lv_st7796.h>
// C++ stdlib
#include <algorithm>
#include <chrono>
#include <initializer_list>
#include <mutex>
#include <cstddef>
#include <cstring>
#include <utility>
#include <vector>
#include <limits>
#include <cinttypes>
// Local/project headers
// #include "cxx/util.hpp"
// #include "bsp/usec.hpp"

#pragma endregion "Include dependencies"
// -----------------------------------------------------------------------------
#pragma region "Definitions"

// Code intended to run in ISR context is placed in IRAM (instruction RAM) for
// optimal performance.
#define fastcode IRAM_ATTR /* __attribute__((section(".iram1.text"))) */

#define log_trace(...) \
  bsp::log_write("%s\r\n\t%s:%d | ", __PRETTY_FUNCTION__, __FILE__, __LINE__); \
  bsp::log_write(__VA_ARGS__); \
  bsp::log_write("\r\n")

#pragma endregion "Definitions"
// -----------------------------------------------------------------------------
#pragma region "User-defined literals"

using namespace std::literals::chrono_literals;

#pragma endregion "User-defined literals"
// -----------------------------------------------------------------------------

#pragma endregion "Preprocessor macros"
//
// =============================================================================

// =============================================================================
//
#pragma region "Board-support package API"

#if NUM_TASKS_TRACED

extern "C" {
  extern SemaphoreHandle_t sync_lvgl_task;
  extern SemaphoreHandle_t sync_stats_task;
  extern void stats_task(void *arg);
}

#endif

namespace bsp {

// -----------------------------------------------------------------------------
#pragma region "Type traits"

// template <typename>   struct return_type;
// template <typename R> struct return_type<R()> { using type = R; };

#pragma endregion "Type traits"
// -----------------------------------------------------------------------------
#pragma region "Type aliases"

// template <typename T>
// using return_type_t = typename return_type<T>::type;

// using ticks_t = std::chrono::duration<TickType_t, std::ratio<1, configTICK_RATE_HZ>>;

#pragma endregion "Type aliases"
// -----------------------------------------------------------------------------
#pragma region "Global constants"

// TAG is required by the ESP-IDF logging system.
static inline constexpr char TAG[] = "ESP32-3248S035";

struct LVGLTask {
  static constexpr std::size_t size = FREERTOS_LVGL_TASK_STACK_SIZE;
  static inline StackType_t stack[size] = { 0 };
  static inline StaticTask_t task {};
};

#pragma endregion "Global constants"
// -----------------------------------------------------------------------------
#pragma region "Utility classes"

// struct Tick : std::chrono::duration<TickType_t, std::ratio<1, configTICK_RATE_HZ>> {

//   Tick() = default;
//   Tick(const Tick &) = default;
//   Tick(Tick &&) = default;
//   Tick &operator=(const Tick &) = default;
//   Tick &operator=(Tick &&) = default;

//   // Convert TickType_t. This enables implicit conversion to and from the native
//   // FreeRTOS type (used throughout the ESP-IDF SDK).
//   Tick(const TickType_t &ticks): duration(ticks) {}
//   inline operator TickType_t() const noexcept { return count(); }

//   // Convert arbitrary duration. These enables implicit conversion to and from:
//   //  - Any of the standard duration types (e.g., std::chrono::seconds), and
//   //  - Any of the std::literals::chrono_literals (e.g., 1s, 200ms, 3h, etc.).

//   template <typename Rep, typename Period>
//   Tick(const std::chrono::duration<Rep, Period> &dur)
//     : duration(std::chrono::duration_cast<duration>(dur)) {}
//   template <typename Rep, typename Period>
//   inline operator std::chrono::duration<Rep, Period>() const
//     { return std::chrono::duration_cast<std::chrono::duration<Rep, Period>>(*this); }
// };

#pragma endregion "Utility classes"
// -----------------------------------------------------------------------------
#pragma region "Pure abstract interfaces"

// Critical provides a mutual exclusion mechanism for shared resources.
//
// It is intended to be used as a base class for singleton classes that need to
// protect access to the hardware peripherals they represent.
struct Critical : std::mutex {
  using Section = const std::lock_guard<Critical::mutex>;

  // By using lock() from a derived context — which uses *this as the mutex —
  // the derived class can prevent concurrent access to any number of critical
  // sections (CSs) nested within otherwise unrelated methods (e.g., methods
  // that utilize the same hardware resource in methods Tx() and Rx()).
  virtual inline Section lock()
    { return Section(*this); }

  // If a derived class needs to guard a critical section (CS) based on some
  // other mutex — i.e., prevent concurrent access to its own CS while still
  // permitting concurrent access to other CSs —  it only needs to declare a
  // Critical member variable, and use that variable's lock() method instead of
  // the inherited lock() method.
};

template <typename ...T>
struct Initer {
  virtual esp_err_t init(T... t) = 0;
};

struct Updater {
  virtual esp_err_t update(TickType_t const) = 0;
};

template <typename ...T>
struct Controller : Critical, Initer<T...>, Updater {
  virtual TickType_t refresh_rate() const noexcept = 0;
};

struct View : Controller<lv_obj_t *> {
  virtual std::string title() = 0;
};

#pragma endregion "Pure abstract interfaces"
// -----------------------------------------------------------------------------
#pragma region "Utility functions"

// Combine words at compile-time using bitwise operators.
//
// These can increase readability, since each operand is an argument and does
// not require parentheses to enforce precedence.
//
// Additionally, these can be used to express operands (and result) as another
// type, which is necessary when their type's bitwise operator is undefined;
// e.g., enum class{} elems, different types (std::size_t|std::uint32_t), etc.
template <typename T, typename ...U>
inline constexpr auto  or_bits(U... u)
  { return (static_cast<T>(u) | ...); }
template <typename T, typename ...U>
inline constexpr auto and_bits(U... u)
  { return (static_cast<T>(u) & ...); }

// Return the N'th least-addressable byte in an unsigned integral value.
template <std::size_t N, typename T,
  std::enable_if<std::is_integral_v<T>, T>* = nullptr>
inline constexpr std::uint8_t byte_at(T value)
  { return static_cast<std::uint8_t>(value >> (N << 3)); }

// Log messages to stdout or stderr (on, e.g., the default serial port).
template <typename ...U> inline void log_write(U... u)
  { fprintf(stdout, u...); }
template <typename ...U> inline void log_error(U... u)
  { fprintf(stderr, u...); }

// Return the number of milliseconds since boot.
static inline std::uint32_t millis() { return esp_timer_get_time() / 1000; }
static inline TickType_t ticks() { return pdMS_TO_TICKS(millis()); }

// Convert a CPU core ID to affinity.
static inline esp_intr_cpu_affinity_t core_affinity(const BaseType_t core_id)
  { return static_cast<esp_intr_cpu_affinity_t>(core_id + 1); }

#pragma endregion "Utility functions"
// -----------------------------------------------------------------------------
#pragma region "Hardware abstractions"

// struct Microseconds
// {
//   using type = return_type_t<decltype(esp_timer_get_time)>;

//   virtual ~Microseconds() = delete;

//   template <typename MillisType = std::uint32_t,
//     std::enable_if_t<std::is_integral_v<MillisType>, int> = 0>
//   static constexpr type from_milliseconds(const MillisType msecs) noexcept {
//     return static_cast<type>(msecs) * 1000LL;
//   }

//   template <typename MillisType = std::uint32_t,
//     std::enable_if_t<std::is_integral_v<MillisType>, int> = 0>
//   static constexpr MillisType to_milliseconds(const type usecs) noexcept {
//     return static_cast<MillisType>(usecs) / 1000LL;
//   }

//   static constexpr type from_ticks(const TickType_t ticks) noexcept {
//     return from_milliseconds(pdTICKS_TO_MS(ticks));
//   }

//   static constexpr TickType_t to_ticks(const type usecs) noexcept {
//     return pdMS_TO_TICKS(to_milliseconds(usecs));
//   }

//   static inline type uptime() noexcept {
//     return esp_timer_get_time();
//   }

//   template <typename MillisType = std::uint32_t,
//     std::enable_if_t<std::is_integral_v<MillisType>, int> = 0>
//   static inline MillisType uptime_milliseconds() noexcept {
//     return to_milliseconds(uptime());
//   }

//   static inline TickType_t uptime_ticks() noexcept {
//     return to_ticks(uptime());
//   }

//   static inline type elapsed(type &since) {
//     const type begin = since;
//     return (since = uptime()) - begin;
//   }

//   template <typename MillisType = std::uint32_t,
//     std::enable_if_t<std::is_integral_v<MillisType>, int> = 0>
//   static inline MillisType elapsed_milliseconds(type &since) {
//     return to_milliseconds(elapsed(since));
//   }

//   static inline TickType_t elapsed_ticks(type &since) {
//     return to_ticks(elapsed(since));
//   }
// };

// struct TIM : Critical, Initer<const bool, const bool>
// {
//   TIM(
//     const std::string &name, // Unique name to give timer
//     const esp_timer_cb_t &cb, // Periodic callback
//     // Optional parameters
//     void *arg = nullptr, // Argument passed to callback
//     const Microseconds::type timeout = default_timeout
//   )
//   : _timeout(timeout),
//     _args(esp_timer_create_args_t{
//       .callback = cb,
//       .arg = arg,
//       .dispatch_method = dispatch_method,
//       .name = name.c_str(),
//       .skip_unhandled_events = false,
//     }),
//     _handle(nullptr) {}

//   virtual ~TIM() {
//     if (_handle != nullptr) {
//       ESP_ERROR_CHECK(
//         esp_timer_delete(_handle)
//       );
//     }
//   }

//   virtual esp_err_t init(
//     const bool begin = true, const bool one_shot = false
//   ) override {
//     {
//       // auto token = lock();
//       ESP_ERROR_CHECK(
//         esp_timer_create(&_args, &_handle)
//       );
//     }
//     if (begin) {
//       return resume(one_shot);
//     }
//     return ESP_OK;
//   }

//   inline bool is_active() {
//     // auto token = lock();
//     return esp_timer_is_active(_handle);
//   }

//   inline esp_err_t resume(const bool one_shot = false) {
//     if (is_active()) {
//       return ESP_OK;
//     }
//     return start(_timeout, one_shot);
//   }

//   inline esp_err_t restart(
//     const Microseconds::type timeout,
//     const bool one_shot = false
//   ) {
//     if (is_active()) {
//       //auto token = lock();
//       //_timeout = timeout;
//       //return esp_timer_restart(_handle, _timeout);
//       stop();
//     }
//     return start(timeout, one_shot);
//   }

//   inline esp_err_t stop() {
//     if (!is_active()) {
//       return ESP_OK;
//     }
//     // auto token = lock();
//     return esp_timer_stop(_handle);
//   }

// protected:
//   inline esp_err_t start(
//     const Microseconds::type timeout,
//     const bool one_shot = false
//   ) {
//     if (is_active()) {
//       return ESP_OK;
//     }
//     // auto token = lock();
//     _timeout = timeout;
//     if (one_shot) {
//       return esp_timer_start_once(_handle, _timeout);
//     }
//     return esp_timer_start_periodic(_handle, _timeout);
//   }

// protected:
//   static constexpr auto dispatch_method =
// #ifdef CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
//     ESP_TIMER_ISR
// #else
//     ESP_TIMER_TASK
// #endif
//   ;

//   static constexpr auto default_timeout =
//     Microseconds::from_ticks(LCD_TICK_RATE);

//   Microseconds::type _timeout;
//   esp_timer_create_args_t _args;
//   esp_timer_handle_t _handle;
// };

struct PWM : Critical, Initer<> {
public:
  using level_t = std::uint8_t;
  static constexpr level_t min_level = std::numeric_limits<level_t>::min();
  static constexpr level_t max_level = std::numeric_limits<level_t>::max();
  static constexpr float min_ratio = static_cast<float>(min_level);
  static constexpr float max_ratio = static_cast<float>(max_level);
  static constexpr level_t default_level = 0x0;

public:
  PWM(
    const gpio_num_t pin, // GPIO pin number
    const ledc_channel_t chan, // PWM channel
    const ledc_timer_bit_t bits, // PWM resolution
    const std::uint32_t freq_hz, // PWM frequency
    // Optional parameters
    const level_t level = default_level,
    const std::uint32_t duty = 0,
    const int hpoint = 0,
    const bool invert = false,
    const bool actlow = false,
    const ledc_intr_type_t intr = LEDC_INTR_DISABLE,
    const ledc_clk_cfg_t cc = LEDC_AUTO_CLK
  )
  : _channel(ledc_channel_config_t{
      .gpio_num = pin,
      .speed_mode = static_cast<ledc_mode_t>(chan >> 3),
      .channel = static_cast<ledc_channel_t>(chan & 7),
      .intr_type = intr,
      .timer_sel = static_cast<ledc_timer_t>((chan >> 1) & 3),
      .duty = duty,
      .hpoint = hpoint,
      .flags = { .output_invert = invert },
    }),
    _timer(ledc_timer_config_t{
      .speed_mode = static_cast<ledc_mode_t>(chan >> 3),
      .duty_resolution = bits,
      .timer_num = static_cast<ledc_timer_t>((chan >> 1) & 3),
      .freq_hz = freq_hz,
      .clk_cfg = cc,
      .deconfigure = false,
    }),
    _active_low(actlow),
    _level(level) {}

  virtual ~PWM() {
    ESP_ERROR_CHECK_WITHOUT_ABORT(
      ledc_stop(_channel.speed_mode, _channel.channel, 0)
    );
    ledc_timer_config_t timer = _timer;
    timer.freq_hz = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(
      ledc_timer_config(&timer)
    );
  }

  virtual esp_err_t init() override {
    ESP_RETURN_ON_ERROR(
      ledc_timer_config(&_timer),
      TAG, "failed to configure PWM timer"
    );
    ESP_RETURN_ON_ERROR(
      ledc_channel_config(&_channel),
      TAG, "failed to configure PWM channel"
    );
    ESP_RETURN_ON_ERROR(
      ledc_fade_func_install(0),
      TAG, "failed to initialize PWM fade service"
    );
    gpio_matrix_out(
      _channel.gpio_num,
      ledc_periph_signal[_channel.speed_mode].sig_out0_idx + _channel.channel,
      _channel.flags.output_invert,
      _active_low
    );
    return set(_level);
  }

public:
  std::uint32_t duty(const level_t level) const noexcept {
    // Scale level to resolution, e.g.:
    //   8-bit=[0,255] → 10-bit=[0,1023] <==> (level * 1023) / 255
    return (static_cast<std::uint32_t>(level) * ((1 << _timer.duty_resolution) - 1)) /
      ((1 << std::numeric_limits<level_t>::digits) - 1);
  }

  esp_err_t set(const level_t level) {
    ESP_RETURN_ON_ERROR(
      ledc_set_duty(_channel.speed_mode, _channel.channel, duty(level)),
      TAG, "failed to set PWM duty"
    );
    ESP_RETURN_ON_ERROR(
      ledc_update_duty(_channel.speed_mode, _channel.channel),
      TAG, "failed to update PWM duty"
    );
    _level = level;
    return ESP_OK;
  }

  // Set the backlight level as a ratio from 0.0 (off) to 1.0 (max brightness).
  esp_err_t set(const float ratio) {
    return set(static_cast<level_t>(
      ratio * (max_ratio - min_ratio) + min_ratio + 0.5f // naïve rounding
    ));
  }

  // esp_err_t fade(const int level) {
  //   // create lambda callback for fade completion
  //   ledc_cbs_t = {
  //     .fade_cb = [](const ledc_cb_param_t *param, void *user_arg) -> bool {
  //       return true;
  //     }
  //   };
  //   _token = new Critical::Section(*this);
  //   ledc_cb_register(_channel.speed_mode, _channel.channel, &cbs, _token);
  //   ESP_RETURN_ON_ERROR(
  //     ledc_set_fade_with_time(
  //       _channel.speed_mode,
  //       _channel.channel,
  //       duty(level),
  //       LEDC_FADE_TIME,
  //     ),
  //     TAG, "failed to set PWM fade"
  //   );
  // }

protected:
  const ledc_channel_config_t _channel;
  const ledc_timer_config_t _timer;
  const bool _active_low;
  level_t _level;
};

struct SPI : Critical, Initer<> {
public:
  SPI(
    const spi_host_device_t host, // SPI host device
    const gpio_num_t miso, // MISO pin
    const gpio_num_t mosi, // MOSI pin
    const gpio_num_t sclk, // SCLK pin
    // Optional parameters
    const std::size_t txsize = 0,
    const spi_common_dma_t dmach  = SPI_DMA_CH_AUTO
  )
  : _host(host),
    _config((spi_bus_config_t){
      .mosi_io_num = mosi,
      .miso_io_num = miso,
      .sclk_io_num = sclk,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .data4_io_num = -1,
      .data5_io_num = -1,
      .data6_io_num = -1,
      .data7_io_num = -1,
      .max_transfer_sz = static_cast<int>(txsize),
      .flags = 0, // SPICOMMON_BUSFLAG_MASTER|SPICOMMON_BUSFLAG_NATIVE_PINS,
      .isr_cpu_id = core_affinity(esp_cpu_get_core_id()),
      .intr_flags = 0,
    }),
    _dma_channel(dmach) {}

  virtual ~SPI() {
    Critical::Section token = lock();
    if (spi_bus_get_attr(_host) != nullptr) {
      ESP_ERROR_CHECK_WITHOUT_ABORT(
        spi_bus_free(_host)
      );
    }
  }

  virtual esp_err_t init() override {
    Critical::Section token = lock();
    ESP_RETURN_ON_ERROR(
      spi_bus_initialize(_host, &_config, _dma_channel),
      TAG, "failed to initialize SPI bus"
    );
    return ESP_OK;
  }

public:
  spi_host_device_t host() const noexcept { return _host; }
  spi_bus_config_t config() const noexcept { return _config; }

protected:
  const spi_host_device_t _host;
  const spi_bus_config_t _config;
  const spi_common_dma_t _dma_channel;
};

struct I2C : Critical, Initer<> {
public:
  I2C(
    const i2c_port_t port, // I²C port
    const gpio_num_t sda, // SDA pin
    const gpio_num_t scl, // SCL pin
    // Optional parameters
    const TickType_t timeout = pdMS_TO_TICKS(LCD_I2C_TIMEOUT.count()),
    const i2c_mode_t mode = I2C_MODE_MASTER,
    const std::uint32_t clock = LCD_I2C_FREQ_HZ
  )
  : _port(port),
    _config((i2c_config_t){
      .mode = mode,
      .sda_io_num = sda,
      .scl_io_num = scl,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master = { .clk_speed = clock },
      .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
    }),
    _timeout(timeout) {}

  virtual ~I2C() {
    Critical::Section token = lock();
    ESP_ERROR_CHECK_WITHOUT_ABORT(
      i2c_driver_delete(_port)
    );
  }

  virtual esp_err_t init() override {
    Critical::Section token = lock();
    ESP_RETURN_ON_ERROR(
      i2c_param_config(_port, &_config),
      TAG, "failed to configure I²C parameters"
    );
    ESP_RETURN_ON_ERROR(
      i2c_driver_install(_port, _config.mode, 0, 0, 0),
      TAG, "failed to install I²C driver"
    );
    return ESP_OK;
  }

public:
  i2c_port_t port() const noexcept { return _port; }
  i2c_config_t config() const noexcept { return _config; }

  esp_err_t reg_rx(
    const std::uint8_t dev_addr,
    const std::uint16_t reg_addr,
    std::uint8_t *rx_buff,
    const std::size_t rx_size
  ) {
    std::uint8_t reg_addr_msb[2] = {
      byte_at<1>(reg_addr),
      byte_at<0>(reg_addr)
    };
    ESP_RETURN_ON_ERROR(
      i2c_master_write_read_device(
        _port, dev_addr, reg_addr_msb, 2, rx_buff, rx_size, _timeout
      ),
      TAG, "failed to read register from I²C device"
    );
    return ESP_OK;
  }

  template <const std::uint8_t... Tx>
  esp_err_t reg_tx(
    const std::uint8_t dev_addr,
    const std::uint16_t reg_addr
  ) {
    const std::size_t tx_size = sizeof...(Tx) + 2;
    const std::uint8_t tx_buff[tx_size] = {
      byte_at<1>(reg_addr),
      byte_at<0>(reg_addr),
      Tx...
    };
    ESP_RETURN_ON_ERROR(
      i2c_master_write_to_device(
        _port, dev_addr, tx_buff, tx_size, _timeout
      ),
      TAG, "failed to write register to I²C device"
    );
    return ESP_OK;
  }

protected:
  const i2c_port_t _port;
  const i2c_config_t _config;
  const TickType_t _timeout;
};

#pragma endregion "Hardware abstractions"
// -----------------------------------------------------------------------------
#pragma region "Hardware peripheral singletons"

// Probably the reason you bought this cheap ass board — the nice big capacitive
// touch screen saddled onto an ESP32 workhorse.
//
// This class drives both display (ST7796S) and touch controller (GT911).
struct LCD : Critical, Initer<View *> {
public:
  // The maximum size (bytes) of an SPI transaction (Rx/Tx).
  static constexpr std::size_t spi_transfer_size = (LV_COLOR_DEPTH >> 3) *
    LCD_PHY_RES_X * LCD_PHY_RES_Y / LCD_SPI_TX_NDIV;

  // Parameters for the GT911 touch controller.
  static constexpr std::uint16_t i2c_reg_prod_id = 0x8140;
  static constexpr std::size_t i2c_size_prod_id = 4U;
  static constexpr std::uint16_t i2c_reg_stat = 0x814E; // buffer/track status
  static constexpr std::uint16_t i2c_reg_base_point = 0x814F;
  static constexpr std::size_t i2c_touch_max = 5U; // simultaneous touch points

protected:
  using ioctrl_t = esp_lcd_panel_io_handle_t;
  using ioconf_t = esp_lcd_panel_io_spi_config_t;
  using notify_t = esp_lcd_panel_io_event_data_t *;
  using driver_t = lv_display_t *;
  using finger_t = lv_indev_t *;
  using buffer_t = lv_color_t *;

  struct __attribute__((packed)) Touch {
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
    std::uint8_t track;
    std::uint16_t x;
    std::uint16_t y;
    std::uint16_t area;
    // 1 pad byte makes 64 bits in (packed) struct
    std::uint8_t _[1]; // Value is irrelevant
  };

  // Static function wrappers for LCD instance methods.
  // These can be used as callback functions for the various C APIs.
  // Be sure to initialize the *instance member! :)
  template <typename T = LCD>
  struct Callback {
  public:
    static esp_err_t init(T *target) {
      if (target != nullptr) {
        Critical::Section token = target->lock();
        instance = const_cast<T *>(target);
        return ESP_OK;
      }
      return ESP_ERR_INVALID_ARG;
    }

  protected:
    // The following static callback functions should not be called directly.
    //
    // Instead, they are invoked by the ESP-IDF LCD component driver in response
    // to hardware events and high-level API calls.
    // static fastcode void on_timer(void *arg) {
    //   instance->on_timer(arg);
    // }
    static fastcode void run(void *arg) {
      ESP_ERROR_CHECK(
        instance->run(arg)
      );
    }
    static fastcode void on_tx_command(
      lv_display_t *disp, const std::uint8_t *cmd, std::size_t cmd_size,
      const std::uint8_t *param, std::size_t param_size
    ) {
      ESP_ERROR_CHECK_WITHOUT_ABORT(
        instance->on_tx_command(disp, cmd, cmd_size, param, param_size)
      );
    }
    static fastcode void on_tx_color(
      lv_display_t *disp, const std::uint8_t *cmd, std::size_t cmd_size,
      std::uint8_t *param, std::size_t param_size
    ) {
      ESP_ERROR_CHECK_WITHOUT_ABORT(
        instance->on_tx_color(disp, cmd, cmd_size, param, param_size)
      );
    }
    static fastcode bool on_tx_color_flush(ioctrl_t ioctrl, notify_t notify, void *context) {
      return instance->on_tx_color_flush(ioctrl, notify, context);
    }
    static fastcode void on_touch_read(
      lv_indev_t *indev, lv_indev_data_t *data
    ) {
      instance->on_touch_read(indev, data);
    }

  protected:
    // Because the *instance member is such a critical requirement when using
    // this Callback wrapper class — the BSP cannot function without it — the
    // static functions always assume it is non-null. Otherwise, the application
    // will very appropriately crash.
    //
    // A good way to guarantee the above is to call the static Callback::init()
    // function anywhere in the LCD constructor. It is guaranteed because all of
    // of the callbacks are installed in the LCD::init() instance method.
    static inline T *instance = nullptr;

    // Expose the static functions _only_ to the callbacks' target class.
    friend T;
  };

public:
  LCD()
  : // _tim(nullptr),
    _spi(
      new SPI(
        LCD_SPI_HOST_ID,
        static_cast<gpio_num_t>(LCD_SPI_PIN_SDI),
        static_cast<gpio_num_t>(LCD_SPI_PIN_SDO),
        static_cast<gpio_num_t>(LCD_SPI_PIN_SCK),
        spi_transfer_size
      )
    ),
    _i2c(
      new I2C(
        LCD_I2C_PORT_ID,
        static_cast<gpio_num_t>(LCD_I2C_PIN_SDA),
        static_cast<gpio_num_t>(LCD_I2C_PIN_SCL)
      )
    ),
    _backlight(
      new PWM(
        static_cast<gpio_num_t>(LCD_PWM_PIN_BLT),
        static_cast<ledc_channel_t>(LCD_PWM_CHAN_BLT),
        static_cast<ledc_timer_bit_t>(LCD_PWM_BITS_BLT),
        static_cast<std::uint32_t>(LCD_PWM_FREQ_BLT)
      )
    ),
    _ioctrl(nullptr),
    _ioconf(ioconf_t{
      .cs_gpio_num = static_cast<gpio_num_t>(LCD_SPI_PIN_SEL),
      .dc_gpio_num = static_cast<gpio_num_t>(LCD_SPI_PIN_SDC),
      .spi_mode = 0,
      .pclk_hz = LCD_SPI_FREQ_HZ,
      .trans_queue_depth = LCD_SPI_DMA_QSZ,
      .on_color_trans_done = Callback<LCD>::on_tx_color_flush,
      .user_ctx = this,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
      .flags = {
        .dc_high_on_cmd = 0,
        .dc_low_on_data = 0,
        .dc_low_on_param = 0,
        .octal_mode = 0,
        .quad_mode = 0,
        .sio_mode = 0,
        .lsb_first = 0,
        .cs_high_active = 0,
      }
    }),
    _driver(nullptr),
    _finger(nullptr),
    _buffer{nullptr, nullptr} {
    ESP_ERROR_CHECK(
      (Callback<LCD>::init(this))
    );
  }

  virtual ~LCD() {
    // if (_tim != nullptr) {
    //   delete _tim;
    // }
    if (_i2c != nullptr) {
      delete _i2c;
    }
    if (_spi != nullptr) {
      delete _spi;
    }
    if (_backlight != nullptr) {
      delete _backlight;
    }
    if (_finger != nullptr) {
      lv_indev_delete(_finger);
      _finger = nullptr;
    }
    if (_ioctrl != nullptr) {
      _ioctrl->del(_ioctrl);
      _ioctrl = nullptr;
    }
    if (_driver != nullptr) {
      lv_display_delete(_driver);
      _driver = nullptr;
    }
    for (int i = 0; i < 2; i++) {
      if (_buffer[i] != nullptr) {
        heap_caps_free(_buffer[i]);
        _buffer[i] = nullptr;
      }
    }
  }

  virtual esp_err_t init(View *view) override {

    log_trace("initializing LCD sou^H^Hubsystem");

    lv_init();

    // log_trace("creating LCD timer");

    // _tim = new TIM("LCDTimer", Callback<LCD>::on_timer);

    // ESP_RETURN_ON_FALSE(
    //   _tim != nullptr, ESP_ERR_INVALID_STATE,
    //   TAG, "failed to create LCD timer"
    // );

    // log_trace("initializing LCD timer");

    // ESP_RETURN_ON_ERROR(
    //   _tim->init(true, false),
    //   TAG, "failed to initialize LCD timer"
    // );

    log_trace("installing core tick hook");

    lv_tick_set_cb(millis);

    ESP_RETURN_ON_FALSE(
      _backlight != nullptr, ESP_ERR_INVALID_STATE,
      TAG, "invalid PWM interface for LCD backlight"
    );

    log_trace("initializing LCD backlight");

    ESP_RETURN_ON_ERROR(
      _backlight->init(),
      TAG, "failed to initialize LCD backlight"
    );

    log_trace("initializing SPI bus");

    ESP_RETURN_ON_ERROR(
      _spi->init(),
      TAG, "invalid SPI interface for LCD"
    );

    log_trace("initializing I²C bus");

    ESP_RETURN_ON_ERROR(
      _i2c->init(),
      TAG, "invalid I²C interface for LCD"
    );

    log_trace("attaching LCD to SPI bus");

    ESP_RETURN_ON_ERROR(
      esp_lcd_new_panel_io_spi(
        // For some reason, esp_lcd_spi_bus_handle_t is an alias for void*, even
        // though internally it is used as an enum value (which is what we pass
        // as argument as well (before the cast)). Seems like a bug in ESP-IDF.
        (esp_lcd_spi_bus_handle_t)(_spi->host()),
        &_ioconf, &_ioctrl),
      TAG, "failed to attach LCD to SPI bus"
    );

    log_trace("creating LCD display driver");

    _driver = lv_st7796_create(LCD_PHY_RES_X, LCD_PHY_RES_Y, LV_LCD_FLAG_NONE,
      Callback<LCD>::on_tx_command, Callback<LCD>::on_tx_color);

    ESP_RETURN_ON_FALSE(
      _driver != nullptr, ESP_ERR_INVALID_STATE,
      TAG, "failed to create LCD display driver"
    );

    log_trace("allocating LCD framebuffers");

    for (int i = 0; i < 2; i++) {
      _buffer[i] = static_cast<lv_color_t *>(
        heap_caps_malloc(spi_transfer_size, LCD_SPI_DMA_CAP)
      );
      ESP_RETURN_ON_FALSE(
        _buffer[i] != nullptr, ESP_ERR_NO_MEM,
        TAG, "failed to allocate LCD framebuffer"
      );
    }

    log_trace("installing LCD framebuffers");

    lv_display_set_color_format(_driver, LCD_COLOR_FMT);
    lv_display_set_buffers(
      _driver,
      _buffer[0],
      _buffer[1],
      spi_transfer_size,
      LV_DISPLAY_RENDER_MODE_PARTIAL
    );

    log_trace("initializing LCD root view");

    ESP_RETURN_ON_ERROR(
      view->init(lv_screen_active()),
      TAG, "failed to initialize LCD root view"
    );

    log_trace("creating RTOS thread for LCD");

    lv_memset(LVGLTask::stack, 0, FREERTOS_LVGL_TASK_STACK_SIZE);
    _thread = xTaskCreateStaticPinnedToCore(
      Callback<LCD>::run,
      FREERTOS_LVGL_TASK_NAME,
      FREERTOS_LVGL_TASK_STACK_SIZE,
      view,
      FREERTOS_LVGL_TASK_PRIORITY,
      LVGLTask::stack,
      &LVGLTask::task,
      FREERTOS_LVGL_TASK_CORE_ID
    );

    ESP_RETURN_ON_FALSE(
      nullptr != _thread, ESP_ERR_INVALID_STATE,
      TAG, "failed to create RTOS thread for LCD"
    );

    log_trace("creating touch input driver");

    _finger = lv_indev_create();

    ESP_RETURN_ON_FALSE(
      _finger != nullptr, ESP_ERR_INVALID_STATE,
      TAG, "failed to create touch input driver"
    );

    log_trace("initializing touch input driver");

    lv_indev_set_type(_finger, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(_finger, Callback<LCD>::on_touch_read);

    log_trace("successfully initialized LCD, turning display on");

    set_backlight(0.75f);

    return ESP_OK;
  }

  // Call refresh as frequently as possible to update the display. It will block
  // until the next refresh cycle is due, and then it will update the display.
  // While blocking, it allows other tasks and interrupts (e.g., idle task and
  // touch input) to progress concurrently.
  inline esp_err_t refresh(void *arg) {
    static auto last = xTaskGetTickCount();
    View *view = static_cast<View *>(arg);
  #ifndef TASK_SCHED_GREEDY
    if /* conditionally run iff time has elapsed */
  #endif
    (xTaskDelayUntil(&last, view->refresh_rate())) {
      log_trace("refreshing timer (lv_tick_get() = %lu)", lv_tick_get());
      lv_timer_handler();
      ESP_RETURN_ON_ERROR(
        view->update(ticks()),
        TAG, "failed to update view"
      );
    }
    return ESP_OK;
  }

  // inline void on_timer(void *arg) {
  //   static auto last = Microseconds::uptime();
  //   lv_tick_inc(Microseconds::elapsed_milliseconds(last));
  // }

  inline esp_err_t run(void *arg) {
  #if NUM_TASKS_TRACED
    log_trace("waiting for sync_lvgl_task semaphore");
    xSemaphoreTake(sync_lvgl_task, portMAX_DELAY);
  #endif
    for (;;) {
      ESP_RETURN_ON_ERROR(
        refresh(arg),
        TAG, "failed to run LCD task"
      );
    }
  }

  inline esp_err_t on_tx_command(
    lv_display_t *disp, const std::uint8_t *cmd, std::size_t cmd_size,
    const std::uint8_t *param, std::size_t param_size
  ) {
    int tx_cmd = 0;
    lv_memcpy(&tx_cmd, cmd, std::min(static_cast<std::size_t>(sizeof(int)), cmd_size));
    ESP_RETURN_ON_ERROR(
      esp_lcd_panel_io_tx_param(_ioctrl, tx_cmd, param, param_size),
      TAG, "failed to transmit command to LCD"
    );
    return ESP_OK;
  }

  inline esp_err_t on_tx_color(
    lv_display_t *disp, const std::uint8_t *cmd, std::size_t cmd_size,
    std::uint8_t *param, std::size_t param_size
  ) {
    int tx_cmd = 0;
    lv_memcpy(&tx_cmd, cmd, std::min(static_cast<std::size_t>(sizeof(int)), cmd_size));
    lv_draw_sw_rgb565_swap(param, param_size);
    ESP_RETURN_ON_ERROR(
      esp_lcd_panel_io_tx_color(_ioctrl, tx_cmd, param, param_size),
      TAG, "failed to transmit color to LCD"
    );
    return ESP_OK;
  }

  inline bool on_tx_color_flush(ioctrl_t ioctrl, notify_t notify, void *context) {
    lv_display_flush_ready(_driver);
    return false; // Whether a high-priority task was scheduled.
  }

  inline void on_touch_read(lv_indev_t *indev, lv_indev_data_t *data) {
    static Touch last = { .track = 0, .x = 0, .y = 0, .area = 0, ._ = {0} };
    std::size_t count = count_touches();
    switch (count) {
      case 1: {
        Touch pt;
        if (ESP_OK == map_touches(&pt, 1)) {
          // log_trace("mapped to: %d %d", pt.x, pt.y);
          last = pt;
          data->state = LV_INDEV_STATE_PRESSED;
          data->point = { .x = last.x, .y = last.y };
        }
        break;
      }
      default: {
        // log_trace("releasing touch");
        data->state = LV_INDEV_STATE_RELEASED;
        data->point = { .x = last.x, .y = last.y };
        break;
      }
    }
    static auto prev = data->state;
    auto curr = data->state;
    if (prev != curr) {
      log_trace(
        "[%d] %10s @ (%d, %d)",
        count,
        (data->state == LV_INDEV_STATE_PRESSED ? "pressed" : "released"),
        data->point.x,
        data->point.y
      );
      prev = curr;
    }
  }

protected:
  inline std::size_t count_touches() {
    static std::uint8_t stat = 0;
    std::size_t count = 0;

    // log_trace("reading touch status");

    ESP_ERROR_CHECK_WITHOUT_ABORT(
      _i2c->reg_rx(LCD_I2C_DEV_ADD, i2c_reg_stat, &stat, sizeof(stat))
    );
    // log_trace("raw touch stat: %d", stat);

    count = static_cast<std::size_t>(stat) & 0xFFul;
    stat = 0;

    if (((count & 0x80ul) > 0) && ((count & 0x0Ful) < i2c_touch_max)) {
      ESP_ERROR_CHECK_WITHOUT_ABORT(
        _i2c->reg_tx<0>(LCD_I2C_DEV_ADD, i2c_reg_stat)
      );
      count &= 0x0Ful;
    }
    return count;
  }

  inline esp_err_t map_touches(Touch *const pt, const std::size_t count) {
    static constexpr std::size_t max_size = sizeof(Touch) * i2c_touch_max;
    static std::uint8_t rx_buff[max_size];

    lv_display_rotation_t rotation = lv_display_get_rotation(_driver);
    std::int32_t width_px = lv_display_get_horizontal_resolution(_driver);
    std::int32_t height_px = lv_display_get_vertical_resolution(_driver);

    lv_memset(rx_buff, 0, max_size);

    const std::size_t rx_size = std::min(max_size, static_cast<std::size_t>(sizeof(Touch)) * count);
    esp_err_t err = _i2c->reg_rx(
      LCD_I2C_DEV_ADD, i2c_reg_base_point, rx_buff, rx_size
    );
    lv_memcpy(pt, rx_buff, rx_size);

    // log_trace("raw touch data: x=%d y=%d", pt->x, pt->y);

    switch (rotation) {
      case LV_DISPLAY_ROTATION_0: {
        for (std::uint8_t i = 0; i < count; ++i) {
          pt[i].x = width_px - pt[i].x;
          pt[i].y = height_px - pt[i].y;
        }
        break;
      }
      case LV_DISPLAY_ROTATION_90: {
        std::uint16_t swap;
        for (std::uint8_t i = 0; i < count; ++i) {
          swap = pt[i].x;
          pt[i].x = pt[i].y;
          pt[i].y = height_px - swap;
        }
        break;
      }
      case LV_DISPLAY_ROTATION_180: {
        for (std::uint8_t i = 0; i < count; ++i) {
          pt[i].x = pt[i].x;
          pt[i].y = pt[i].y;
        }
        break;
      }
      case LV_DISPLAY_ROTATION_270: {
        std::uint16_t swap;
        for (std::uint8_t i = 0; i < count; ++i) {
          swap = pt[i].x;
          pt[i].x = width_px - pt[i].y;
          pt[i].y = swap;
        }
        break;
      }
    }

    // log_trace("mapped touch data: x=%d y=%d", pt->x, pt->y);

    return err;
  }

public:
  // Set the backlight level in the range [0=OFF, 255=MAX].
  inline esp_err_t set_backlight(const PWM::level_t level) {
    return _backlight->set(level);
  }

  // Set the backlight level as a ratio in the range [0.0=OFF, 1.0=MAX].
  inline esp_err_t set_backlight(const float ratio) {
    return _backlight->set(ratio);
  }

  inline lv_display_t *display() { return _driver; }

protected:
  // TIM *_tim;
  SPI *_spi;
  I2C *_i2c;
  PWM *_backlight;

  TaskHandle_t _thread;

  ioctrl_t _ioctrl;
  ioconf_t _ioconf;
  driver_t _driver;
  finger_t _finger;
  buffer_t _buffer[2];
};

#pragma endregion "Hardware peripheral singletons"
// -----------------------------------------------------------------------------

} // namespace bsp

#pragma endregion "Board-support package API"
//
// =============================================================================
