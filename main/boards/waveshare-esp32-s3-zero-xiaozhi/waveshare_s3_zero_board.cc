#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "config.h"
#include "application.h"
#include "device_state.h"
#include "led/led.h"

#include <driver/gpio.h>
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
        xTaskCreate(PulseTask, "StatusLed", 2048, this, tskIDLE_PRIORITY + 1, nullptr);
    }

    void OnStateChanged() override {
        const auto state = Application::GetInstance().GetDeviceState();
        if (state == kDeviceStateListening || state == kDeviceStateAudioTesting) {
            mode_.store(1);
        } else if (state == kDeviceStateSpeaking || state == kDeviceStateNotifying) {
            mode_.store(2);
        } else {
            // Standby, processing/connecting, and all other states: LED off.
            mode_.store(0);
        }
    }

private:
    gpio_num_t gpio_;
    std::atomic<int> mode_{0};

    static void PulseTask(void* arg) {
        auto* self = static_cast<WaveshareStatusLed*>(arg);
        for (;;) {
            const int mode = self->mode_.load();
            if (mode == 1) {
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
