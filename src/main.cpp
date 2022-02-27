#include <Arduino.h>


#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico_uart_transports.h"

#include "maker_pi_pins.h"


void task_led_0(void *p) {
    while (1) {
        digitalWrite(maker_pi::GROOVE_1_B, HIGH);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        digitalWrite(maker_pi::GROOVE_1_B, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void task_led_1(void *p) {
    while (1) {
        printf("bbbbb\n");
        digitalWrite(maker_pi::GROOVE_1_A, HIGH);
        vTaskDelay(333 / portTICK_PERIOD_MS);
        digitalWrite(maker_pi::GROOVE_1_A, LOW);
        vTaskDelay(333 / portTICK_PERIOD_MS);
    }
}



int main(void) {
  Serial.begin(115200); 

  pinMode(maker_pi::GROOVE_1_A, OUTPUT);
  pinMode(maker_pi::GROOVE_1_B, OUTPUT);

  digitalWrite(maker_pi::GROOVE_1_A, LOW);
  digitalWrite(maker_pi::GROOVE_1_B, LOW);

  rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

  rcl_timer_t timer;
  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;

  allocator = rcl_get_default_allocator();

  // Wait for agent successful ping for 2 minutes.
  const int timeout_ms = 1000; 
  const uint8_t attempts = 120;

  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  if (ret != RCL_RET_OK)
  {
      // Unreachable agent, exiting program.
      return ret;
  }

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "f1tenth_hardware", "", &support);

  rclc_subscription_init_default(
  &motor_vel_subscriber,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  "motor_vel");

  rclc_subscription_init_default(
  &steering_angle_subscriber,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  "steering_ang");

  rclc_publisher_init_default(
  &publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  "enc_tic");

  rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(1000),
      timer_callback);

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &motor_vel_subscriber, &vel_msg, &motor_vel_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &steering_angle_subscriber, &ang_msg, &steering_angle_callback, ON_NEW_DATA);

  xTaskCreate(task_led_0, "task_led_0", 256, NULL, 1, NULL);
  xTaskCreate(task_led_1, "task_led_1", 256, NULL, 1, NULL);
  vTaskStartScheduler();

  while(1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}