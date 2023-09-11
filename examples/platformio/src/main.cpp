#include <ESP323248S035.h>

// If using the lvgl logging system (LV_USE_LOG), you must:
//   1. Define a TPC_LCD::Log function, and
//   2. Assign it to the static log member of the TPC_LCD class.
// Both steps are shown here:
#if (LV_USE_LOG)
const TPC_LCD::Log TPC_LCD::log = [](const char *message) {
  Serial.println(message);
};
#endif

// The ESP32-3248S035 library defines View as a pure virtual class, which acts
// as an interface between the library and application GUI code.
//
// We must define a concrete implementation of View to act as the root window
// of the target device's GUI. The content of View is application-defined.
//
// NOTE: Do NOT make any lvgl API calls from the View constructor. These must be
//       performed in the View::init() method.
class Main: public View {
public:
  Main() = default;
  bool init(lv_obj_t *root) override {
    if (nullptr == root) {
      // Use the default screen if no root view provided.
      root = lv_scr_act();
    }
    lv_obj_t *label = lv_label_create(root);
    lv_obj_center(label);
    lv_label_set_text(label, "Hello, Arduino!");
    return true;
  }
  void update(msecu32_t const now) override {
    // Update any subviews here.
  }
  inline std::string title() override { return "Main"; }
};

static Main root;
//static ESP323248S035C target(root); // C++17 can deduce template parameter.
static ESP323248S035C<Main> target(root); // Pre-C++17 requires explicit type.

// The Arduino API requires a setup() and loop(). These are where you initiate
// and refresh the GUI elements and all other hardware peripherals.
void setup() {
  target.init();
  // The individual hardware peripherals can be accessed via template parameter
  // on method hw<T>(), where T is the peripheral type:
  target.hw<RGB_PWM>().fabulous(0xFF, 5);
}

void loop() {
  target.update();
  delay(2);
}
