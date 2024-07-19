#pragma once

#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include "../cxx/util.hpp"

namespace bsp {

struct Microseconds
{
  using type = cxx::return_type_t<decltype(esp_timer_get_time)>;

  virtual ~Microseconds() = delete;

  template <typename MillisType = std::uint32_t,
    std::enable_if_t<std::is_integral_v<MillisType>, int> = 0>
  static constexpr type from_milliseconds(const MillisType msecs) noexcept {
    return static_cast<type>(msecs) * 1000LL;
  }

  template <typename MillisType = std::uint32_t,
    std::enable_if_t<std::is_integral_v<MillisType>, int> = 0>
  static constexpr MillisType to_milliseconds(const type usecs) noexcept {
    return static_cast<MillisType>(usecs) / 1000LL;
  }

  static constexpr type from_ticks(const TickType_t ticks) noexcept {
    return from_milliseconds(pdTICKS_TO_MS(ticks));
  }

  static constexpr TickType_t to_ticks(const type usecs) noexcept {
    return pdMS_TO_TICKS(to_milliseconds(usecs));
  }

  static inline type uptime() noexcept {
    return esp_timer_get_time();
  }

  template <typename MillisType = std::uint32_t,
    std::enable_if_t<std::is_integral_v<MillisType>, int> = 0>
  static inline MillisType uptime_milliseconds() noexcept {
    return to_milliseconds(uptime());
  }

  static inline TickType_t uptime_ticks() noexcept {
    return to_ticks(uptime());
  }

  static inline type elapsed(type &since) {
    const type begin = since;
    return (since = uptime()) - begin;
  }

  template <typename MillisType = std::uint32_t,
    std::enable_if_t<std::is_integral_v<MillisType>, int> = 0>
  static inline MillisType elapsed_milliseconds(type &since) {
    return to_milliseconds(elapsed(since));
  }

  static inline TickType_t elapsed_ticks(type &since) {
    return to_ticks(elapsed(since));
  }
};

} // namespace cxx
