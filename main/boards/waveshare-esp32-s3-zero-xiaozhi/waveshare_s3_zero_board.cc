#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "config.h"
#include "application.h"
#include "device_state.h"
#include "led/led.h"

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>

class WaveshareStatusLed : public Led {
public:
    explicit WaveshareStatusLed(gpio_num_t gpio) : gpio_(gpio) {
        gpio_config_t config = {};
        config.pin_bit_mask = 1ULL << gpio_;
        config.mode = GPIO_MODE_OUTPUT;
        config.pull_up_en = GPIO_PULLUP_DISABLE;
        config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        config.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&config));
        ESP_ERROR_CHECK(gpio_set_level(gpio_, 0));
        ESP_LOGI("WaveshareStatusLed", "Status LED GPIO %d configured as active-high output", (int)gpio_);

        // Perform the five boot flashes synchronously so they cannot be lost
        // if task creation is delayed or fails during early startup.
        for (int flash = 0; flash < 5; ++flash) {
            ESP_ERROR_CHECK(gpio_set_level(gpio_, 1));
            vTaskDelay(pdMS_TO_TICKS(120));
            ESP_ERROR_CHECK(gpio_set_level(gpio_, 0));
            vTaskDelay(pdMS_TO_TICKS(120));
        }

        const BaseType_t task_result =
            xTaskCreate(PulseTask, "StatusLed", 4096, this, 5, nullptr);
        if (task_result != pdPASS) {
            ESP_LOGE("WaveshareStatusLed", "Failed to create status LED task (result=%ld)",
                     (long)task_result);
        } else {
            ESP_LOGI("WaveshareStatusLed", "Status LED task started");
        }
        mode_.store(0);
    }

    void OnStateChanged() override {
        const auto state = Application::GetInstance().GetDeviceState();
        if (state == kDeviceStateConnecting || state == kDeviceStateListening ||
            state == kDeviceStateAudioTesting) {
            // Light the status LED immediately after Jarvis is accepted, including
            // the connecting phase before the server starts the listening session.
            mode_.store(1);
        } else if (state == kDeviceStateSpeaking || state == kDeviceStateNotifying) {
            mode_.store(2);
        } else {
            // Do not let the early boot/activation idle callback cancel the
            // five-flash startup pattern. A real wake/listening state above
            // still interrupts it immediately.
            int startup_mode = 3;
            mode_.compare_exchange_strong(startup_mode, 0);
        }
    }

private:
    gpio_num_t gpio_;
    std::atomic<int> mode_{3}; // Run five startup flashes before entering standby.

    static void PulseTask(void* arg) {
        auto* self = static_cast<WaveshareStatusLed*>(arg);
        for (;;) {
            const int mode = self->mode_.load();
            if (mode == 3) {
                // Five short flashes at boot. Stop immediately if a real device
                // state arrives while the startup pattern is running.
                for (int flash = 0; flash < 5 && self->mode_.load() == 3; ++flash) {
                    gpio_set_level(self->gpio_, 1);
                    vTaskDelay(pdMS_TO_TICKS(120));
                    gpio_set_level(self->gpio_, 0);
                    vTaskDelay(pdMS_TO_TICKS(120));
                }
                int expected = 3;
                self->mode_.compare_exchange_strong(expected, 0);
            } else if (mode == 1) {
                gpio_set_level(self->gpio_, 1);
                vTaskDelay(pdMS_TO_TICKS(40));
            } else if (mode == 2) {
                gpio_set_level(self->gpio_, 1);
                vTaskDelay(pdMS_TO_TICKS(180));
                gpio_set_level(self->gpio_, 0);
                vTaskDelay(pdMS_TO_TICKS(180));
            } else {
                gpio_set_level(self->gpio_, 0);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
    }
};

class WaveshareS3ZeroBoard : public WifiBoard {
public:
    WaveshareS3ZeroBoard() {
        // MAX98357A SD_MODE is an enable/shutdown input. Do not leave it
        // floating: drive it high before the audio codec is initialized.
        gpio_config_t amp_sd_config = {};
        amp_sd_config.pin_bit_mask = 1ULL << AUDIO_I2S_AMP_SD_GPIO;
        amp_sd_config.mode = GPIO_MODE_OUTPUT;
        amp_sd_config.pull_up_en = GPIO_PULLUP_DISABLE;
        amp_sd_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        amp_sd_config.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&amp_sd_config));
        ESP_ERROR_CHECK(gpio_set_level(AUDIO_I2S_AMP_SD_GPIO, 1));
        ESP_LOGI("WaveshareS3ZeroBoard",
                 "MAX98357A SD_MODE enabled on GPIO %d",
                 (int)AUDIO_I2S_AMP_SD_GPIO);

        // Construct the LED immediately during board startup, rather than
        // waiting for the first application state-change callback.
        (void)GetLed();
    }

    Led* GetLed() override {
        static WaveshareStatusLed led(BUILTIN_LED_GPIO);
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
};

DECLARE_BOARD(WaveshareS3ZeroBoard);
