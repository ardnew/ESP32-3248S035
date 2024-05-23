#pragma once

// =============================================================================
//
#pragma region "Preprocessor macros"

// -----------------------------------------------------------------------------
#pragma region "Include dependencies"

// ESP-IDF SDK
#include <esp_check.h>
#include <esp_err.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/spi_master.h>
#include <driver/spi_common.h>
#include <driver/spi_common_internal.h>
#include <soc/ledc_periph.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
// External libraries
#include <lvgl.h>
#include <src/drivers/display/st7796/lv_st7796.h>
#include <cronos.hpp>
// C++ stdlib
#include <algorithm>
#include <chrono>
#include <mutex>
#include <cstddef>
#include <cstring>
#include <utility>
#include <vector>
#include <limits>

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

#pragma endregion "Preprocessor macros"
//
// =============================================================================

// =============================================================================
//
#pragma region "Board-support package API"

namespace bsp {

// -----------------------------------------------------------------------------
#pragma region "Type aliases"

using Thread = TaskHandle_t;

#pragma endregion "Type aliases"
// -----------------------------------------------------------------------------
#pragma region "Global constants"

// TAG is required by the ESP-IDF logging system.
static inline constexpr char TAG[] = "ESP32-3248S035";

#pragma endregion "Global constants"
// -----------------------------------------------------------------------------
#pragma region "Pure abstract interfaces"

// Critical provides a mutual exclusion mechanism for shared resources.
//
// It is intended to be used as a base class for singleton classes that need to
// protect access to the hardware peripherals they represent.
struct Critical : std::mutex {
  using Section = std::scoped_lock<Critical::mutex>;
  virtual inline const Section lock() { return Section(*this); }
};

template <typename ...T>
struct Initer {
  virtual esp_err_t init(T... t) = 0;
};

struct Updater {
  virtual void update(msecu32_t const) = 0;
};

template <typename ...T>
struct Controller : Critical, Initer<T...>, Updater {
  virtual std::uint32_t refresh_hz() const noexcept = 0;
};

struct View : Controller<lv_obj_t *> {
  virtual std::string title() = 0;
};

#pragma endregion "Pure abstract interfaces"
// -----------------------------------------------------------------------------
#pragma region "Utility functions"

// Combine words using bitwise operators.
//
// These can increase readability, since each operand is an argument and does
// not require parentheses to enforce precedence.
template <typename ...U> inline constexpr auto  or_bits(U... u) { return (u | ...); }
template <typename ...U> inline constexpr auto and_bits(U... u) { return (u & ...); }

// Log messages to stdout or stderr (on, e.g., the default serial port).
template <typename ...U> inline void log_write(U... u) { fprintf(stdout, u...); }
template <typename ...U> inline void log_error(U... u) { fprintf(stderr, u...); }

// In-place byte swap for an array of elements.
template <typename T>
void byte_swap(const T *p, std::size_t n) {
    for (std::size_t head = 0, tail = n - 1;  head < n / 2;  ++head, --tail) {
        std::swap(((std::uint8_t *)p)[head], ((std::uint8_t *)p)[tail]);
    }
}

void print_buffer(const void *buf, std::size_t len, const bool eol = true, const char *fsz = "[%lu] ", const char *fmt = "%02X ") {
  std::size_t total = std::min(len, static_cast<std::size_t>(32));
  std::size_t limit = len > total ? total / 2 : len;
  printf(fsz, len);
  for (std::size_t i = 0; i < limit; ++i) {
    printf(fmt, static_cast<const std::uint8_t *>(buf)[i]);
  }
  if (len > total) {
    printf(" ..<%lu>.. ", len - total);
    for (std::size_t i = len - limit; i < len; ++i) {
      printf(fmt, static_cast<const std::uint8_t *>(buf)[i]);
    }
  }
  if (eol) { printf("\n"); }
}

#pragma endregion "Utility functions"
// -----------------------------------------------------------------------------
#pragma region "Hardware abstractions"

struct PWM : Critical, Initer<> {
public:
  using level_t = std::uint8_t;
  static constexpr level_t default_level = 0x80;

protected:
  const ledc_channel_config_t _channel;
  const ledc_timer_config_t _timer;
  const bool _active_low;
  level_t _level;

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

  static inline constexpr level_t max_level() noexcept
    { return std::numeric_limits<level_t>::max(); }
  static inline constexpr level_t min_level() noexcept
    { return std::numeric_limits<level_t>::min(); }
};

struct SPI : Critical, Initer<> {
public:
  struct Device : Initer<std::function<esp_err_t(Device *)>> {
    spi_device_handle_t handle;
    spi_device_interface_config_t config;
    gpio_num_t cs_pin;
    gpio_num_t dc_pin;
    Device(
      gpio_num_t cs_pin, // Chip-select (CS) pin
      gpio_num_t dc_pin, // Data/command (DC) or register select (RS) pin
      int freq_hz, // SPI bus frequency
      int pool_size, // "In-flight" transaction queue size
      transaction_cb_t pre_tx, // Pre-transaction callback
      transaction_cb_t post_tx, // Post-transaction callback
      // Optional parameters
      std::uint8_t mode = 0, // Mode 0
      std::uint32_t flags = SPI_DEVICE_HALFDUPLEX|SPI_DEVICE_NO_DUMMY,
      std::uint8_t command = 8,
      std::uint8_t address = 8,
      std::uint8_t dummy = 0,
      std::uint16_t duty = 0,
      std::uint16_t wait = 0,
      std::uint8_t hold = 0,
      int valid = 0
    )
    : handle(nullptr),
      config({
        .command_bits = command,
        .address_bits = address,
        .dummy_bits = dummy,
        .mode = mode,
        .duty_cycle_pos = duty,
        .cs_ena_pretrans = wait,
        .cs_ena_posttrans = hold,
        .clock_speed_hz = freq_hz,
        .input_delay_ns = valid,
        .spics_io_num = static_cast<int>(cs_pin),
        .flags = flags,
        .queue_size = pool_size,
        .pre_cb = pre_tx,
        .post_cb = post_tx,
      }),
      cs_pin(cs_pin),
      dc_pin(dc_pin) {}

    virtual ~Device() {
      ESP_ERROR_CHECK_WITHOUT_ABORT(
        spi_bus_remove_device(handle)
      );
    }

    virtual esp_err_t init(
      std::function<esp_err_t(Device *)> init_func
    ) override {
      return init_func(this);
    }
  };

  struct Queue : Initer<> {
    using item_t = spi_transaction_t;
    using item_ptr_t = item_t *;
    using item_ptr_ref_t = item_ptr_t &;

    Queue(std::size_t size)
    : _queue(xQueueCreate(size, sizeof(item_ptr_t))),
      _capacity(size) {}

    virtual ~Queue() {
      if (_queue != nullptr) {
        vQueueDelete(_queue);
      }
    }

    virtual esp_err_t init() override {
      for (std::size_t i = 0; i < _capacity; ++i) {
        item_ptr_t pit = static_cast<item_ptr_t>(
          heap_caps_malloc(sizeof(item_t), LCD_SPI_DMA_CAP)
        );
        ESP_RETURN_ON_FALSE(
          pit != nullptr,
          ESP_ERR_NO_MEM,
          TAG, "failed to allocate SPI transaction"
        );
        memset(pit, 0, sizeof(item_t));
        send(pit, 0);
      }
      return ESP_OK;
    }

    esp_err_t send(item_ptr_ref_t p, TickType_t ticks) {
      return xQueueSend(_queue, &p, ticks);
    }

    esp_err_t receive(item_ptr_ref_t p, TickType_t ticks) {
      return xQueueReceive(_queue, &p, ticks);
    }

    bool is_full() const noexcept {
      return uxQueueMessagesWaiting(_queue) == 0;
    }

    void yield(SPI::Device *dev, std::size_t count) {
      item_ptr_t pit = nullptr;
      while (uxQueueMessagesWaiting(_queue) < count) {
        if (spi_device_get_trans_result(dev->handle, &pit, 1) == ESP_OK) {
          send(pit, 0);
        }
      }
    }

  protected:
    QueueHandle_t _queue;
    std::size_t _capacity;
  };

protected:
  const spi_host_device_t _host;
  const spi_bus_config_t _config;
  const spi_common_dma_t _dma_channel;

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
    _config(spi_bus_config_t{
      .mosi_io_num = mosi,
      .miso_io_num = miso,
      .sclk_io_num = sclk,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = static_cast<int>(txsize),
      .flags = SPICOMMON_BUSFLAG_MASTER,
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

  esp_err_t add(Device *dev) {
    Critical::Section token = lock();
    return spi_bus_add_device(_host, &(dev->config), &(dev->handle));
  }

  spi_bus_config_t config() const noexcept { return _config; }
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
    LCD_PHY_RES_X * LCD_PHY_RES_Y / LCD_SPI_TX_REPS;

  // Context objects attached to the (void*)user field of spi_transaction_t.
  // Used for attaching arbitrary data to a transaction.
  struct Context {
    // Mode controls the signal level of DC/RS. This line is connected to an
    // ordinary GPIO pin and changed during pre-transfer callback (pre_tx).
    enum class Mode : std::uint32_t {
      Command = 0,
      Data
    };

    // Method controls how the SPI transaction is performed.
    enum class Method : std::uint8_t {
      Sync  = 0b001, // MIPI commands and short data
      Async = 0b010, // Long, multi-buffered color/data windows
      Flush = 0b100  // Cleanup and notify lvgl the transaction is complete
    };

    const Mode mode;
    const Method method;

    constexpr Context(
      const Mode mode = Mode::Command,
      const Method method = Method::Sync
    )
    : mode(mode), method(method) {}

    template <typename ...M,
      std::enable_if_t<(std::is_same_v<M, Method> && ...), int> = 0>
    constexpr Context(
      const Mode mode = Mode::Command,
      M... m
    )
    : mode(mode),
      method(static_cast<Method>(or_bits(static_cast<std::uint8_t>(m) ...))) {}

    constexpr Context(
      const Context &ctx
    )
    : mode(ctx.mode), method(ctx.method) {}

    virtual ~Context() = default;

    template <typename ...M,
      std::enable_if_t<(std::is_same_v<M, Method> && ...), int> = 0>
    bool uses(M... m) const noexcept {
      return or_bits(static_cast<std::uint8_t>(m) ...) &
        static_cast<std::uint8_t>(method);
    }
  };

  std::size_t queued = 0;
  std::size_t flushed = 0;

protected:
  Thread _task;
  SPI *_spi;
  SPI::Device *_device;
  SPI::Queue *_queue;
  PWM *_backlight;
  lv_display_t *_display;
  lv_color_t *_fb[2];

public:
  LCD()
  : _spi(
      new SPI(
        LCD_SPI_HOST_ID,
        static_cast<gpio_num_t>(LCD_SPI_PIN_SDI),
        static_cast<gpio_num_t>(LCD_SPI_PIN_SDO),
        static_cast<gpio_num_t>(LCD_SPI_PIN_SCK),
        spi_transfer_size
      )
    ),
    _device(
      new SPI::Device(
        static_cast<gpio_num_t>(LCD_SPI_PIN_SEL),
        static_cast<gpio_num_t>(LCD_SPI_PIN_SDC),
        static_cast<int>(LCD_SPI_FREQ_HZ),
        static_cast<int>(LCD_SPI_TX_POOL),
        Callback<LCD, Context>::pre_tx,
        Callback<LCD, Context>::post_tx
      )
    ),
    _queue(
      new SPI::Queue(
        LCD_SPI_TX_POOL
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
    _display(nullptr),
    _fb{nullptr, nullptr} {
    ESP_ERROR_CHECK(
      (Callback<LCD, Context>::init(this))
    );
  }

  virtual ~LCD() {
    if (_task != nullptr) {
      vTaskDelete(_task);
    }
    if (_spi != nullptr) {
      delete _spi;
    }
    if (_device != nullptr) {
      delete _device;
    }
    if (_queue != nullptr) {
      delete _queue;
    }
    if (_backlight != nullptr) {
      delete _backlight;
    }
    if (_display != nullptr) {
      lv_display_delete(_display);
      free(_display);
    }
    for (int i = 0; i < 2; i++) {
      if (_fb[i] != nullptr) {
        heap_caps_free(_fb[i]);
      }
    }
  }

  virtual esp_err_t init(View *view) override {

    log_trace("initializing LCD sou^H^Hubsystem");

    lv_init();

    lv_tick_set_cb(xTaskGetTickCount);

    log_trace("initializing GPIO pins");

    gpio_config_t io_conf = {
      .pin_bit_mask = or_bits(
        1ULL << LCD_SPI_PIN_SEL, // CS
        1ULL << LCD_SPI_PIN_SDC, // DC/RS
        1ULL << LCD_PWM_PIN_BLT  // Backlight
      ),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(static_cast<gpio_num_t>(LCD_SPI_PIN_SDC), 1); // DC/RS: data mode
    gpio_set_level(static_cast<gpio_num_t>(LCD_PWM_PIN_BLT), 0); // Backlight: off
    gpio_set_level(static_cast<gpio_num_t>(LCD_SPI_PIN_SEL), 0); // CS: active

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

    log_trace("attaching LCD device to SPI bus");

    ESP_RETURN_ON_ERROR(
      _device->init([&](SPI::Device *dev) {
        return _spi->add(dev);
      }),
      TAG, "failed to attach LCD device to SPI bus"
    );

    log_trace("initializing SPI transaction queue");

    ESP_RETURN_ON_ERROR(
      _queue->init(),
      TAG, "failed to initialize SPI transaction queue"
    );

    log_trace("creating LCD display driver");

    _display = lv_st7796_create(
      LCD_PHY_RES_X,
      LCD_PHY_RES_Y,
      LV_LCD_FLAG_NONE,
      Callback<LCD, Context>::cmd,
      Callback<LCD, Context>::color
    );

    ESP_RETURN_ON_FALSE(
      _display != nullptr, ESP_ERR_INVALID_STATE,
      TAG, "failed to create LCD display driver"
    );

    log_trace("allocating LCD framebuffers");

    for (int i = 0; i < 2; i++) {
      _fb[i] = static_cast<lv_color_t *>(
        heap_caps_malloc(spi_transfer_size, LCD_SPI_DMA_CAP)
      );
      ESP_RETURN_ON_FALSE(
        _fb[i] != nullptr, ESP_ERR_NO_MEM,
        TAG, "failed to allocate LCD framebuffer"
      );
    }

    log_trace("installing LCD framebuffers");

    lv_display_set_buffers(
      _display,
      _fb[0],
      _fb[1],
      spi_transfer_size,
      LV_DISPLAY_RENDER_MODE_PARTIAL
    );

    log_trace("initializing LCD root view");

    ESP_RETURN_ON_ERROR(
      view->init(nullptr),
      TAG, "failed to initialize LCD root view"
    );

    log_trace("creating RTOS thread for LCD");

    ESP_RETURN_ON_FALSE(
      pdPASS == xTaskCreate(
        Callback<LCD, Context>::task,
        FREERTOS_LVGL_TASK_NAME,
        FREERTOS_LVGL_TASK_STACK_SIZE,
        view,
        FREERTOS_LVGL_TASK_PRIORITY,
        &_task
      ), ESP_ERR_INVALID_STATE,
      TAG, "failed to create RTOS thread for LCD"
    );

    log_trace("successfully initialized LCD");

    return ESP_OK;
  }

  inline void task(void *arg) {
    // xTaskDelayUntil arguments are expressed in units of system "ticks".
    // Conventionally, 1 tick = 1 millisecond regardless of CPU frequency.
    View *view = static_cast<View *>(arg);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
  #ifndef TASK_SCHED_GREEDY
      if /* conditionally run func() iff time has elapsed */
  #endif
      (xTaskDelayUntil(&last, view->refresh_hz())) {
        lv_timer_handler();
        view->update(msecu32());
      }
    }
  }

  inline fastcode void pre_tx(spi_transaction_t *trx) {
    // log_trace("pre_tx-transaction callback start");
    gpio_set_level(_device->cs_pin, 0);
    if (trx != nullptr && trx->user != nullptr) {
      Context *ctx = static_cast<Context *>(trx->user);
      gpio_set_level(_device->dc_pin, static_cast<std::uint32_t>(ctx->mode));
    }
    // log_trace("pre_tx-transaction callback end");
  }

  inline fastcode void post_tx(spi_transaction_t *trx) {
    if (trx != nullptr && trx->user != nullptr) {
      // The post_tx-transaction callback is called even during the creation of
      // the display driver (when sending SPI init commands).
      // The _display member will still be uninitialized at this point (only).
      if (_display != nullptr) {
        Context *ctx = static_cast<Context *>(trx->user);
        if (ctx->uses(Context::Method::Flush)) {
          lv_disp_flush_ready(_display);
          auto token = lock();
          ++flushed;
        }
        delete ctx;
      }
      trx->user = nullptr;
      gpio_set_level(_device->cs_pin, 1);
    }
  }

  inline void transmit(
    std::uint8_t *data, std::size_t size, Context ctx = Context()
  ) {
    if (size == 0) { return; }

    spi_transaction_t trx {
      .flags = 0,
      .length = size << 3,
      .user = new Context(ctx)
    };

    if (size <= sizeof(trx.tx_data) && data != nullptr) {
      trx.flags |= SPI_TRANS_USE_TXDATA;
      memcpy(trx.tx_data, data, size);
    } else {
      trx.tx_buffer = data;
    }

    if (ctx.uses(Context::Method::Sync)) {
      const char *fsz = (ctx.mode == Context::Mode::Command) ?
        "S [%lu] " : " S[%lu] ";
      // before polling, all previous pending transactions need to be serviced
      _queue->yield(_device, LCD_SPI_TX_POOL);
      if (trx.flags & SPI_TRANS_USE_TXDATA) {
        print_buffer(trx.tx_data, size, true, fsz);
      } else {
        print_buffer(trx.tx_buffer, size, true, fsz);
      }
      spi_device_polling_transmit(_device->handle, &trx);
    } else {
      const char *fsz = (ctx.mode == Context::Mode::Command) ?
        "A [%lu] " : " A[%lu] ";
      // if necessary, ensure we can queue new transactions by servicing some
      // previous transactions
      if (_queue->is_full()) {
        _queue->yield(_device, LCD_SPI_TX_RESV);
      }

		  spi_transaction_t *pit = nullptr;
      _queue->receive(pit, portMAX_DELAY);
      memcpy(pit, &trx, sizeof(trx));
      if (trx.flags & SPI_TRANS_USE_TXDATA) {
        print_buffer(pit->tx_data, size, true, fsz);
      } else {
        print_buffer(pit->tx_buffer, size, true, fsz);
      }
      if (spi_device_queue_trans(_device->handle, pit, portMAX_DELAY) != ESP_OK) {
        // send failed transaction back to the pool to be reused
        _queue->send(pit, portMAX_DELAY);
      } else {
        auto token = lock();
        ++queued;
      }
    }
  }

  inline SPI *spi() noexcept { return _spi; }
  inline SPI::Device *device() noexcept { return _device; }
  inline SPI::Queue *queue() noexcept { return _queue; }
  inline PWM *backlight() noexcept { return _backlight; }
  inline lv_display_t *display() noexcept { return _display; }
  inline lv_color_t *fb(std::size_t i) noexcept { return _fb[i&1]; }

protected:
  // Static function wrappers for LCD instance methods.
  // These can be used as callback functions for the various C APIs.
  // Be sure to initialize the *instance member! :)
  template <typename T = LCD, typename C = Context>
  struct Callback {
  protected:
    static inline T *instance = nullptr;

  public:
    static esp_err_t init(const T *target) {
      if (target != nullptr) {
        instance = const_cast<T *>(target);
        return ESP_OK;
      }
      return ESP_ERR_INVALID_ARG;;
    }

    static void task(void *arg) {
      if (instance != nullptr) {
        instance->task(arg);
      }
    }

    static fastcode void pre_tx(spi_transaction_t *trx) {
      if (instance != nullptr) {
        instance->post_tx(trx);
      }
    }

    static fastcode void post_tx(spi_transaction_t *trx) {
      if (instance != nullptr) {
        instance->post_tx(trx);
      }
    }

    static inline void cmd(
      lv_display_t *disp, const std::uint8_t *cmd, std::size_t cmd_size,
      const std::uint8_t *param, std::size_t param_size
    ) {
      if (instance != nullptr) {
        spi_device_acquire_bus(
          instance->device()->handle,
          portMAX_DELAY
        );

        //bswap(const_cast<std::uint8_t *>(cmd), 8, cmd_size);
        gpio_set_level(static_cast<gpio_num_t>(LCD_SPI_PIN_SDC), 0);
        gpio_set_level(static_cast<gpio_num_t>(LCD_SPI_PIN_SEL), 0);
        instance->transmit(
          const_cast<std::uint8_t *>(cmd), cmd_size,
          Context(Context::Mode::Command, Context::Method::Sync)
        );
        gpio_set_level(static_cast<gpio_num_t>(LCD_SPI_PIN_SDC), 1);
        //bswap(const_cast<std::uint8_t *>(param), 8, param_size);
        instance->transmit(
          const_cast<std::uint8_t *>(param), param_size,
          Context(Context::Mode::Data, Context::Method::Sync)
        );

        gpio_set_level(static_cast<gpio_num_t>(LCD_SPI_PIN_SEL), 1);
        spi_device_release_bus(instance->device()->handle);
      }
    }

    static inline void color(
      lv_display_t *disp, const std::uint8_t *cmd, std::size_t cmd_size,
      std::uint8_t *param, std::size_t param_size
    ) {
      if (instance != nullptr) {
        spi_device_acquire_bus(
          instance->device()->handle,
          portMAX_DELAY
        );

        //bswap(const_cast<std::uint8_t *>(cmd), 8, cmd_size);
        gpio_set_level(static_cast<gpio_num_t>(LCD_SPI_PIN_SDC), 0);
        gpio_set_level(static_cast<gpio_num_t>(LCD_SPI_PIN_SEL), 0);
        instance->transmit(
          const_cast<std::uint8_t *>(cmd), cmd_size,
          Context(Context::Mode::Command, Context::Method::Sync)
        );
        gpio_set_level(static_cast<gpio_num_t>(LCD_SPI_PIN_SDC), 1);
        for (std::size_t i = 0; i < param_size; i += 2) {
          byte_swap(param + i, 2);
        }
        instance->transmit(
          const_cast<std::uint8_t *>(param), param_size,
          Context(
            Context::Mode::Data,
            Context::Method::Sync,
            Context::Method::Flush
          )
        );

        gpio_set_level(static_cast<gpio_num_t>(LCD_SPI_PIN_SEL), 1);
        spi_device_release_bus(instance->device()->handle);
      }
    }
  };
};

#pragma endregion "Hardware peripheral singletons"
// -----------------------------------------------------------------------------

} // namespace bsp

#pragma endregion "Board-support package API"
//
// =============================================================================
