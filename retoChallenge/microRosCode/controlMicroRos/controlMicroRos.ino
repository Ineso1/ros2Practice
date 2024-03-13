#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#define LED_PIN 10
#define IRQ_FLAG 23
#define IN_PIN_1 12
#define IN_PIN_2 13
#define EN_PIN 33
#define ENCODER_A_PIN 34
#define ENCODER_B_PIN 32

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// publisher
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_w;
rclc_executor_t executor_pub;
rcl_timer_t timer;

// publisher
rcl_publisher_t publisher_target;
std_msgs__msg__Int32 msg_w_target;
rclc_executor_t executor_pub_target;
rcl_timer_t timer_target;

// subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg_signal;
rclc_executor_t executor_sub;

const int ledChannel = 0;
float rpm_Target = 0;
float rpm_Current = 0;
float ww = 0;
float unit_Signal = 0;
int motor_Direction = 0;

const int frequency = 5000;
const int resolution = 8;

int encoderAState = 0;
int encoderBState = 0;

int current_Counter = 0;

double error;
double errores[3] = {0};
int currPwm = 0;
int prevPwm = 0;

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void setVelocity(float target){

  int pwmSpeed;
  error = rpm_Target - rpm_Current;

  errores[1] = errores[0];
  errores[2] = errores[1];
  errores[0] = error;

  prevPwm = currPwm;
  
  double kp = 0.4;
  double ki = 0.5;
  double kd = 0.001;
  double a = (kp + kd/0.02) * errores[0];
  double b = (-kp + (ki*0.02) - (2*kd/0.02)) * errores[1];
  double c = (kd/0.02) * errores[2];

  pwmSpeed = (prevPwm + a + b + c);


  //pwmSpeed = prevPwm + error * kp;
  //prevPwm = currPwm;
  currPwm = pwmSpeed;

  int e_direction;
  if (error <= -4) {
    e_direction = !motor_Direction;
  } else if (error >= 4) {
    e_direction = motor_Direction;
  }

  ledcWrite(ledChannel, currPwm);
  digitalWrite(IN_PIN_1, e_direction == -1 ? HIGH : LOW);
  digitalWrite(IN_PIN_2, e_direction == 1 ? HIGH : LOW);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    rpm_Current = (current_Counter * 600) / (2 * 374);
    msg_w.data = rpm_Current;
    RCSOFTCHECK(rcl_publish(&publisher, &msg_w, NULL));
    current_Counter = 0;
  }
}

void timer_callback_target(rcl_timer_t *timer_target, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer_target != NULL)
  {
    msg_w_target.data = rpm_Target;
    RCSOFTCHECK(rcl_publish(&publisher_target, &msg_w_target, NULL));
  }
}

void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg_signal = (const std_msgs__msg__Float32 *)msgin;
  unit_Signal = msg_signal->data;
  float map_w = unit_Signal * 340;
  if(map_w < 0){
    map_w *= -1;
    motor_Direction = -1; 
    }
  else{ motor_Direction = 1;}
  
  rpm_Target = map_w;
}

void IRAM_ATTR isr() {
  digitalWrite(IRQ_FLAG, !digitalRead(IRQ_FLAG));
  current_Counter++;
}

void setup() {
  set_microros_transports();
  pinMode(IN_PIN_1, OUTPUT);
  pinMode(IN_PIN_2, OUTPUT);
  pinMode(IRQ_FLAG, OUTPUT);
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  ledcSetup(ledChannel, frequency, resolution);
  ledcAttachPin(EN_PIN, ledChannel);
  delay(200);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "control_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "signal"));
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "w"));
  RCCHECK(rclc_publisher_init_default(&publisher_target, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "w_target"));

  const unsigned int timer_timeout = 200;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));
  RCCHECK(rclc_timer_init_default(&timer_target, &support, RCL_MS_TO_NS(timer_timeout), timer_callback_target));

  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_init(&executor_pub_target, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
  RCCHECK(rclc_executor_add_timer(&executor_pub_target, &timer_target));
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_signal, &subscription_callback, ON_NEW_DATA));

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), isr, CHANGE);

  msg_w.data = 0;
  msg_w_target.data = 0;
}

void loop() {
  delay(50);
  setVelocity(rpm_Target);
  RCCHECK(rclc_executor_spin_some(&executor_pub_target, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}
