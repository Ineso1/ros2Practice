#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// publisher
rcl_publisher_t publisher;
std_msgs_msg_Int32 msg_raw_plot;
rclc_executor_t executor_pub;
rcl_timer_t timer;

// publisher 2
rcl_publisher_t publisher_2;
std_msgs_msg_Float32 msg_voltage;
rcl_timer_t timer_2;

// subscriber
rcl_subscription_t subscriber;
std_msgs_msg_Int32 msg_pwm_duty_cycle;
rclc_executor_t executor_sub;

#define LED_PIN_1 12
#define LED_PIN_2 15
#define ADC_PIN 36

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

float dutyCycle;
const int frequency = 5000;
const int resolution = 8;
const int ledChannel = 0;

/**
 * @brief loop to indicate error with blinking LED
 *
 */
void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN_1, !digitalRead(LED_PIN_1));
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    RCSOFTCHECK(rcl_publish(&publisher, &msg_raw_plot, NULL));
    msg_raw_plot.data = analogRead(ADC_PIN);
  }
}

void timer_callback_2(rcl_timer_t *timer_2, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer_2 != NULL)
  {
    RCSOFTCHECK(rcl_publish(&publisher_2, &msg_voltage, NULL));
    msg_voltage.data = map(analogRead(ADC_PIN), 0, 4095, -0.3, 3.3) + 0.3;
  }
}

/**
 * @brief subscription callback executed at receiving a message
 *
 * @param msgin
 */
void subscription_callback(const void *msgin)
{
  const std_msgs_msgFloat32 *msg_pwm_duty_cycle = (const std_msgsmsg_Float32 *)msgin;
  // (condition) ? (true exec):(false exec)
  dutyCycle = msg_pwm_duty_cycle->data;
  ledcWrite(ledChannel, dutyCycle);
}

void setup()
{
  set_microros_transports();

  pinMode(LED_PIN_1, OUTPUT);
  //pinMode(LED_PIN_2, OUTPUT);
  ledcSetup(ledChannel, frequency, resolution);
  ledcAttachPin(LED_PIN_2, ledChannel);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create subscriber
  // const char topic_name_led[] = "xiao_led_state";
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "pwm_duty_cycle"));

  // create publisher
  // const char topic_name_heatbeat[] = "xiao_heartbeat";
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "raw_plot"));
  RCCHECK(rclc_publisher_init_default(
      &publisher_2,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "voltage"));

  // create timer, called every 1000 ms to publish heartbeat
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));
  
  const unsigned int timer_timeout_2 = 500;
  RCCHECK(rclc_timer_init_default(
      &timer_2,
      &support,
      RCL_MS_TO_NS(timer_timeout_2),
      timer_callback_2));

  // create executor
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer_2));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_pwm_duty_cycle, &subscription_callback, ON_NEW_DATA));

  msg_voltage.data = 0.0;
  msg_raw_plot.data = 0;
}

void loop()
{
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}