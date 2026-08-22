#include <cstdio>
#include <cstdint>
#include <cstdlib>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

constexpr int kServoGpio = 13;
constexpr uint32_t kServoFrequencyHz = 50;
constexpr ledc_timer_bit_t kServoResolution = LEDC_TIMER_16_BIT;

constexpr ledc_mode_t kPwmMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kChannel = LEDC_CHANNEL_0;

uint32_t pulse_us_to_duty(uint32_t pulse_us){
    constexpr uint32_t period_us = 1'000'000 / kServoFrequencyHz;
    constexpr uint32_t max_duty = (1U << 16) -1;

    return (static_cast<uint64_t>(pulse_us) * max_duty) / period_us;

}

extern "C" void app_main(void)
{
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
    int x = 500;
    uint32_t goal = 2500;


    while(true){

    uint32_t Servo_duty = pulse_us_to_duty(x);


    ESP_ERROR_CHECK(ledc_set_duty(kPwmMode, kChannel, Servo_duty));
    ESP_ERROR_CHECK(ledc_update_duty(kPwmMode, kChannel));

    ESP_LOGI("SERVO", "Sending %d us pulse; duty count = %" PRIu32, x, Servo_duty);
    
    
    
    if(x < goal){
        x+=5;
        printf("Increasing towards goal");
        if(x > goal - 5){
            goal = 500;
            printf("Goal changed to 500");
        }
    }
    if(goal == 500 && x > goal){
        x-=5;
        printf("Decreasing towards goal");
        if(x < goal + 5){
            goal = 2500;
            printf("Goal changed to 2500");
        }
    } 
    }
}