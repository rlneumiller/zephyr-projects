#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <stdio.h>
#include "led.h"

#define LED_SLEEP_TIME_MS 1000 // Sleep for 1 second
#define LED0_NODE         DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define STACK_SIZE_LED_THREAD 1024
#define PRIORITY_LED_THREAD   10

// Timing constants in milliseconds
#define HEARTBEAT_PULSE 100
#define CYCLE_TIME      1000 // Total cycle time 1 second

void led_thread_proc(void)
{
	printf("Entering LED thread proc\n");
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		printf("Error: LED GPIO port not ready\n");
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printf("Error configuring LED GPIO pin\n");
		return;
	}

	while (1) {
		uint32_t start_time = k_uptime_get_32();
		uint32_t next_time = start_time;

		// First beat
		gpio_pin_set_dt(&led, 1);
		next_time += HEARTBEAT_PULSE;
		k_sleep(K_TIMEOUT_ABS_MS(next_time));

		gpio_pin_set_dt(&led, 0);
		next_time += HEARTBEAT_PULSE;
		k_sleep(K_TIMEOUT_ABS_MS(next_time));

		// Second beat
		gpio_pin_set_dt(&led, 1);
		next_time += HEARTBEAT_PULSE;
		k_sleep(K_TIMEOUT_ABS_MS(next_time));

		gpio_pin_set_dt(&led, 0);
		next_time += HEARTBEAT_PULSE;
		k_sleep(K_TIMEOUT_ABS_MS(next_time));

		// Wait remainder of cycle
		next_time = start_time + CYCLE_TIME;
		k_sleep(K_TIMEOUT_ABS_MS(next_time));
		//printf("Total heartbeat cycle: %dms\n\n", k_uptime_get_32() - start_time);
	}
}

K_THREAD_DEFINE(toggle_led_tid, STACK_SIZE_LED_THREAD, led_thread_proc, NULL, NULL, NULL,
		PRIORITY_LED_THREAD, 0, 0);