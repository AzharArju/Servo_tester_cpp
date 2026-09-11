#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_hidh_api.h"


constexpr int kServoGpio = 26;
constexpr uint32_t kServoFrequencyHz = 50;
constexpr ledc_timer_bit_t kServoResolution = LEDC_TIMER_16_BIT;

constexpr ledc_mode_t kPwmMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kChannel = LEDC_CHANNEL_0;

constexpr uint32_t kMinPulseUs = 500;
constexpr uint32_t kMaxPulseUs = 2500;

static esp_bd_addr_t controller_address = {0x41, 0x42, 0x00, 0x00, 0x0F, 0x22};
static bool connection_requested = false;

static std::atomic<uint32_t> left_stick_y{127};


uint32_t pulse_us_to_duty(uint32_t pulse_us){
    constexpr uint32_t period_us = 1'000'000 / kServoFrequencyHz;
    constexpr uint32_t max_duty = (1U << 16) -1;

    return (static_cast<uint64_t>(pulse_us) * max_duty) / period_us;

}

void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT && param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED)
    ESP_LOGI("BT", "Discovery finished");

    if (event != ESP_BT_GAP_DISC_RES_EVT) return;
    ESP_LOGI("BT", "Discovered a nearby Bluetooth device");

    const uint8_t *address = param->disc_res.bda;
    ESP_LOG_BUFFER_HEX("BT_ADDR", address, ESP_BD_ADDR_LEN);

    const int property_count = param->disc_res.num_prop;
    const esp_bt_gap_dev_prop_t *properties = param->disc_res.prop;
    
    for (int i = 0; i < property_count; ++i) {
        const esp_bt_gap_dev_prop_t &property = properties[i];
        ESP_LOGI("BT", "Property type=%d; length=%d bytes", property.type, property.len);
        if (property.type != ESP_BT_GAP_DEV_PROP_EIR) continue;
        uint8_t *eir = static_cast<uint8_t *>(property.val);
        uint8_t name_length = 0;
        const uint8_t *name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &name_length);
        if (name != nullptr)
        ESP_LOGI("BT", "Device name: %.*s", name_length, reinterpret_cast<const char *>(name));
    }
}

uint32_t find_current_pulse_us(uint32_t left_stick_y) {
    return kMaxPulseUs - (left_stick_y * (kMaxPulseUs - kMinPulseUs) / 255);
}

void hid_callback(esp_hidh_cb_event_t event, esp_hidh_cb_param_t *param)
{
    //ESP_LOGI("BT_HID", "HID event: %d", event);
    if (event == ESP_HIDH_INIT_EVT) {
        ESP_LOGI("BT_HID", "HID initialization status: %d", param->init.status);
    }
    if (event == ESP_HIDH_OPEN_EVT) {
        ESP_LOGI("BT_HID", "Connection result: %d; state: %d", param->open.status, param->open.conn_status);
    }
    if (event == ESP_HIDH_INIT_EVT && param->init.status == ESP_HIDH_OK && !connection_requested) {
    ESP_ERROR_CHECK(esp_bt_hid_host_connect(controller_address));
    connection_requested = true;
    }
    if (event == ESP_HIDH_DATA_IND_EVT && param->data_ind.len >= 10) {
    const uint8_t *data = param->data_ind.data;
    static TickType_t last_joystick_log = 0;
    TickType_t now = xTaskGetTickCount();

    if (now - last_joystick_log >= pdMS_TO_TICKS(100)) {
    ESP_LOGI("STICKS", "LX=%u LY=%u RX=%u RY=%u",
             data[1], data[2], data[3], data[4]);
    last_joystick_log = now;
    left_stick_y.store(data[2]);
    }
    static uint8_t previous_controls[5] = {};

    if (std::memcmp(&data[5], previous_controls, 5) != 0) {
        ESP_LOG_BUFFER_HEX("BUTTONS", &data[5], 5);
        std::memcpy(previous_controls, &data[5], 5);
    }
}
}
    


extern "C" void app_main(void)
{
    esp_err_t result = nvs_flash_init();
    ESP_ERROR_CHECK(result);

    esp_bt_controller_config_t bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_config));

    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));
    ESP_ERROR_CHECK(esp_bluedroid_init());

    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_LOGI("BT", "Bluetooth stack ready");

    ESP_ERROR_CHECK(esp_bt_hid_host_register_callback(hid_callback));
    ESP_ERROR_CHECK(esp_bt_hid_host_init());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_callback));
    //ESP_ERROR_CHECK(esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0));

    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = kPwmMode;
            timer_config.duty_resolution = kServoResolution;
    timer_config.timer_num = kTimer;
    timer_config.freq_hz = kServoFrequencyHz;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = kServoGpio;
    channel_config.speed_mode = kPwmMode;
    channel_config.channel = kChannel;
    channel_config.timer_sel = kTimer;
    channel_config.duty = 0;
    channel_config.hpoint = 0;

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));


    vTaskDelay(pdMS_TO_TICKS(5000));



    while(true){
    
    uint32_t Servo_duty = pulse_us_to_duty(find_current_pulse_us(left_stick_y.load()));

    ESP_ERROR_CHECK(ledc_set_duty(kPwmMode, kChannel, Servo_duty));
    ESP_ERROR_CHECK(ledc_update_duty(kPwmMode, kChannel));

    ESP_LOGI("SERVO", "Sending %d us pulse; duty count = %" PRIu32, find_current_pulse_us(left_stick_y.load()), Servo_duty);

    vTaskDelay(pdMS_TO_TICKS(20));
    }
}