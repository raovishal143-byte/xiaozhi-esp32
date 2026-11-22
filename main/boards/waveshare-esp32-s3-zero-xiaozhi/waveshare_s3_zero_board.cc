#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "config.h"
#include "application.h"
#include "device_state.h"
#include "led/led.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <led_strip.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>

namespace {
constexpr gpio_num_t kOnboardRgbGpio = GPIO_NUM_21;
constexpr gpio_num_t kScannerPins[] = {
    GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_12,
    GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_15
};
constexpr int kScannerLedCount = sizeof(kScannerPins) / sizeof(kScannerPins[0]);
constexpr uint32_t kPwmMax = 8191;
constexpr uint32_t kScannerMain = kPwmMax;
constexpr uint32_t kScannerTrail1 = (kPwmMax * 70) / 100;
constexpr uint32_t kScannerTrail2 = (kPwmMax * 25) / 100;

class WaveshareStatusLed : public Led {
public:
    WaveshareStatusLed() {
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num = kOnboardRgbGpio;
        strip_config.max_leds = 1;
        strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
        strip_config.led_model = LED_MODEL_WS2812;

        led_strip_rmt_config_t rmt_config = {};
        rmt_config.resolution_hz = 10 * 1000 * 1000;
        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip_));

        // Exactly five red startup flashes on the onboard addressable RGB LED.
        for (int i = 0; i < 5; ++i) {
            SetColor(255, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(150));
            Clear();
            vTaskDelay(pdMS_TO_TICKS(150));
        }

        const BaseType_t result = xTaskCreate(StatusTask, "JarvisRgb", 3072, this, 5, nullptr);
        if (result != pdPASS) {
            ESP_LOGE("WaveshareStatusLed", "Could not create RGB status task");
        }
    }

    void OnStateChanged() override {
        const auto state = Application::GetInstance().GetDeviceState();
        if (state == kDeviceStateListening || state == kDeviceStateAudioTesting ||
            state == kDeviceStateConnecting) {
            mode_.store(1);  // Blue blink: listening / processing
        } else if (state == kDeviceStateSpeaking || state == kDeviceStateNotifying) {
            mode_.store(2);  // Green blink: assistant audio
        } else {
            mode_.store(0);
        }
    }

private:
    led_strip_handle_t strip_ = nullptr;
    std::atomic<int> mode_{0};  // Yellow standby by default

    void SetColor(uint8_t red, uint8_t green, uint8_t blue) {
        led_strip_set_pixel(strip_, 0, red, green, blue);
        led_strip_refresh(strip_);
    }

    void Clear() {
        led_strip_clear(strip_);
    }

    static void StatusTask(void* arg) {
        auto* self = static_cast<WaveshareStatusLed*>(arg);
        bool on = false;
        int previous_mode = -1;
        for (;;) {
            const int mode = self->mode_.load();
            if (mode == 0) {
                // Yellow steady: powered on and waiting in standby.
                self->SetColor(255, 160, 0);
                on = true;
                previous_mode = 0;
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            if (mode != previous_mode) {
                on = false;
                previous_mode = mode;
            }
            if (on) {
                self->Clear();
            } else if (mode == 1) {
                self->SetColor(0, 0, 255);
            } else {
                self->SetColor(0, 255, 0);
            }
            on = !on;
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }
};

class KittScanner {
public:
    void Start() {
        ledc_timer_config_t timer = {};
        timer.speed_mode = LEDC_LOW_SPEED_MODE;
        timer.duty_resolution = LEDC_TIMER_13_BIT;
        timer.timer_num = LEDC_TIMER_0;
        timer.freq_hz = 5000;
        timer.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&timer));

        for (int i = 0; i < kScannerLedCount; ++i) {
            ledc_channel_config_t channel = {};
            channel.gpio_num = kScannerPins[i];
            channel.speed_mode = LEDC_LOW_SPEED_MODE;
            channel.channel = static_cast<ledc_channel_t>(i);
            channel.intr_type = LEDC_INTR_DISABLE;
            channel.timer_sel = LEDC_TIMER_0;
            channel.duty = 0;
            channel.hpoint = 0;
            ESP_ERROR_CHECK(ledc_channel_config(&channel));
        }

        const BaseType_t result = xTaskCreate(ScanTask, "KittScanner", 3072, this, 4, nullptr);
        if (result != pdPASS) {
            ESP_LOGE("KittScanner", "Could not create scanner task");
        } else {
            ESP_LOGI("KittScanner", "Six-LED bidirectional scanner started on GPIO 10-15");
        }
    }

private:
    static void Write(int index, uint32_t duty) {
        const auto channel = static_cast<ledc_channel_t>(index);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    }

    static void ScanTask(void* arg) {
        auto* self = static_cast<KittScanner*>(arg);
        (void)self;
        int position = 0;
        int direction = 1;
        for (;;) {
            for (int i = 0; i < kScannerLedCount; ++i) {
                uint32_t duty = 0;
                if (i == position) {
                    duty = kScannerMain;
                } else if (i == position - direction) {
                    duty = kScannerTrail1;
                } else if (i == position - (2 * direction)) {
                    duty = kScannerTrail2;
                }
                Write(i, duty);
            }

            vTaskDelay(pdMS_TO_TICKS(250));
            position += direction;
            if (position >= kScannerLedCount - 1) {
                position = kScannerLedCount - 1;
                direction = -1;
            } else if (position <= 0) {
                position = 0;
                direction = 1;
            }
        }
    }
};
}  // namespace

class WaveshareS3ZeroBoard : public WifiBoard {
public:
    WaveshareS3ZeroBoard() {
        // Initialize the onboard RGB status LED and perform its five red flashes.
        (void)GetLed();

        // Give the startup greeting sequence time to begin before starting the
        // independent scanner animation. The scanner then runs continuously.
        vTaskDelay(pdMS_TO_TICKS(2000));
        scanner_.Start();
    }

    Led* GetLed() override {
        static WaveshareStatusLed led;
        return &led;
    }

    AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK,
            AUDIO_I2S_SPK_GPIO_LRCK,
            AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK,
            AUDIO_I2S_MIC_GPIO_WS,
            AUDIO_I2S_MIC_GPIO_DIN
        );
        return &audio_codec;
    }

private:
    KittScanner scanner_;
};

DECLARE_BOARD(WaveshareS3ZeroBoard);
