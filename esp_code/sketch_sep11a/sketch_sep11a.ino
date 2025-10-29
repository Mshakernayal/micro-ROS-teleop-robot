#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <ESP32Servo.h>

#define LED_PIN 4
#define SERVO1_PIN 12  
#define SERVO2_PIN 14  

rcl_publisher_t pos_pub;
rcl_publisher_t debug_pub;
rcl_subscription_t cmd_sub;

std_msgs__msg__Int32MultiArray pos_msg;
std_msgs__msg__Float32MultiArray debug_msg;
geometry_msgs__msg__Twist cmd_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

Servo servo1, servo2;
int servo1_pos = 90;
int servo2_pos = 90;
float left_speed = 0.0;
float right_speed = 0.0;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear = msg->linear.x;
  float angular = msg->angular.z;


  if (linear > 0.0 && fabs(angular) < 0.01) {
    // Top arrow → Left +, Right -
    left_speed = +1.0;
    right_speed = -1.0;
  } 
  else if (linear < 0.0 && fabs(angular) < 0.01) {
    // Bottom arrow → Left -, Right +
    left_speed = -1.0;
    right_speed = +1.0;
  } 
  else if (angular > 0.0 && fabs(linear) < 0.01) {
    // Left arrow → both +
    left_speed = +1.0;
    right_speed = +1.0;
  } 
  else if (angular < 0.0 && fabs(linear) < 0.01) {
    // Right arrow → both -
    left_speed = -1.0;
    right_speed = -1.0;
  } 
  else {
    // Otherwise stop
    left_speed = 0.0;
    right_speed = 0.0;
  }

  // Map [-1, +1] → [0,180]
  servo1_pos = constrain(map(left_speed * 100, -100, 100, 0, 180), 0, 180);
  servo2_pos = constrain(map(right_speed * 100, -100, 100, 0, 180), 0, 180);

  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {

    pos_msg.data.data[0] = servo1_pos;
    pos_msg.data.data[1] = servo2_pos;
    pos_msg.data.size = 2;
    RCSOFTCHECK(rcl_publish(&pos_pub, &pos_msg, NULL));


    debug_msg.data.data[0] = left_speed;
    debug_msg.data.data[1] = right_speed;
    debug_msg.data.size = 2;
    RCSOFTCHECK(rcl_publish(&debug_pub, &debug_msg, NULL));
  }
}


void setup() {
  set_microros_wifi_transports("POCO X3 NFC", "123456ac", "10.100.215.110", 8888);
  WiFi.setSleep(false);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);


  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);

  delay(100);


  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_diff_drive_node", "", &support));


  RCCHECK(rclc_publisher_init_best_effort(
    &pos_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "rb_position"));

  RCCHECK(rclc_publisher_init_best_effort(
    &debug_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "rb_debug"));


  RCCHECK(rclc_subscription_init_default(
    &cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));


  const unsigned int timer_timeout = 500; 
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_vel_callback, ON_NEW_DATA));


  pos_msg.data.data = (int32_t*)malloc(2 * sizeof(int32_t));
  pos_msg.data.size = 2;
  pos_msg.data.capacity = 2;

  debug_msg.data.data = (float*)malloc(2 * sizeof(float));
  debug_msg.data.size = 2;
  debug_msg.data.capacity = 2;
}


void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
  delay(10); // Helps Wi-Fi stack
}

