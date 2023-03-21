#include "pwm_conf.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

void setup_pin_pwm(uint8_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    // Figure out which slice we just connected to the LED pin
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&config, 1.0);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_num, &config, true);
}