#include "main.hpp"

//#define use_static_lcd_task

#ifndef FREERTOS_LVGL_TASK_NAME
#undef use_static_lcd_task
#endif // FREERTOS_LVGL_TASK_NAME

#ifndef FREERTOS_LVGL_TASK_STACK_SIZE
#undef use_static_lcd_task
#endif // FREERTOS_LVGL_TASK_STACK_SIZE

bsp::LCD *lcd = new bsp::LCD();
Main *view = new Main(lcd);

fastcode void vApplicationTickHook(void) { lv_tick_inc(pdTICKS_TO_MS(1)); }

//esp_register_freertos_tick_hook_for_cpu(esp_freertos_tick_cb_tnew_tick_cb, UBaseType_t cpuid)

void run()
{
  // Allow other core to finish initialization
  vTaskDelay(pdMS_TO_TICKS(100));

#if NUM_TASKS_TRACED // Create semaphores to hold tasks until released
  sync_lvgl_task = xSemaphoreCreateCounting(NUM_TASKS_TRACED, 0);
  sync_stats_task = xSemaphoreCreateBinary();
#endif

  ESP_ERROR_CHECK(
      lcd->init(view));

#if NUM_TASKS_TRACED
  // Create stats task and release all traced tasks
  xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, FREERTOS_LVGL_TASK_PRIORITY, NULL, FREERTOS_LVGL_TASK_CORE_ID+1);// tskNO_AFFINITY);
  xSemaphoreGive(sync_stats_task);
#endif

  // If both task name and stack size are defined, then the LCD class manages
  // running the LVGL task.
#ifndef use_static_lcd_task
  ESP_ERROR_CHECK(
      lcd->run(view));
#endif

  // bsp::LCD::run will never return. If we reach here, then it wasn't called,
  // and the bsp::LCD class is managing our main event loop.
  // We can safely return from app_main.
  vTaskDelete(nullptr);
}

extern "C" void app_main(void)
{
  log_trace("enter app_main");
  run();
  log_trace("exit app_main");
}
