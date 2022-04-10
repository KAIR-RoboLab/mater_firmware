#include <Arduino.h>

// standard libraries
#include <stdio.h>
#include <time.h>
#include <math.h>

// microros dependencies
// #include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

// microros message definitions
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

// external libraries
#include <PID_v1.h>
#include <pio_rotary_encoder.h>

// custom transport layer definition
#include "maker_pi_transport.h"


#define ERROR_LOOP_LED_PIN (16)
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// error loop used to handle exceptions of uROS
void error_loop(){
  Serial.println("Error occured");
  while(1){
    digitalWrite(ERROR_LOOP_LED_PIN, !digitalRead(ERROR_LOOP_LED_PIN));
    delay(100);
  }
}

// initialise static encoder variables for both motors
int RotaryEncoder::rotation_motor_a = 0;
int RotaryEncoder::rotation_motor_b = 0;

// create motor object
RotaryEncoder encoder_left(2, 3, MOTOR_A_SM);
RotaryEncoder encoder_right(26, 27, MOTOR_B_SM);

// allocate space for previous encoder readings
int last_rotation_left;
int last_rotation_right;

// PID gains
#define Kp (0.0)
#define Ki (0.001)
#define Kd (0.0)
#define PWM_MAX_VAL (65535)

// define PID for wheels
double state_left, output_left, setpoint_left;
double state_right, output_right, setpoint_right;

PID pid_left_wheel(&state_left, &output_left, &setpoint_left, Kp, Ki, Kd, AUTOMATIC);
PID pid_right_wheel(&state_right, &output_right, &setpoint_right, Kp, Ki, Kd, AUTOMATIC);


extern int clock_gettime(clockid_t unused, struct timespec *tp);

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t odometry_publisher;
rcl_publisher_t joint_state_publisher;
nav_msgs__msg__Odometry odometry_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
bool micro_ros_init_successful;


enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;



void euler_to_quat(float x, float y, float z, double* q) {
  float c1 = cos((y*PI/180.0)/2.0);
  float c2 = cos((z*PI/180.0)/2.0);
  float c3 = cos((x*PI/180.0)/2.0);

  float s1 = sin((y*PI/180.0)/2.0);
  float s2 = sin((z*PI/180.0)/2.0);
  float s3 = sin((x*PI/180.0)/2.0);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
  q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}


void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_vel_msg.linear.x = msg->linear.x;
  cmd_vel_msg.angular.z = msg->angular.z;
}


void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    // update encode readings
    int current_rotation_left = encoder_left.get_rotation();
    int current_rotation_right = encoder_right.get_rotation();

    // MODIFY CODE BELOW

    setpoint_left = (double) cmd_vel_msg.linear.x;
    state_left = (double) (current_rotation_left - last_rotation_left);
    pid_left_wheel.Compute();
    analogWrite(10, (int) (output_left * PWM_MAX_VAL));

    setpoint_right = (double) cmd_vel_msg.linear.x;
    state_right = (double) (current_rotation_right - last_rotation_right);
    pid_right_wheel.Compute();
    analogWrite(8, (int) (output_right * PWM_MAX_VAL));

    odometry_msg.twist.twist.linear.x = ((double) state_left + (double) state_right) / 2.0;

    // END OF YOUR CODE

    // set current encoder readings to be last ones
    last_rotation_left = current_rotation_left;
    last_rotation_right = current_rotation_right;

    // update time stamp in odometry message
    struct timespec tv = {0};
    clock_gettime(0, &tv);
    odometry_msg.header.stamp.nanosec = tv.tv_nsec;
    odometry_msg.header.stamp.sec = tv.tv_sec;

    // publish odometry message
    RCSOFTCHECK(rcl_publish(&odometry_publisher, &odometry_msg, NULL));
  }
}


bool create_entities() {
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "rmbot_hardware_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &odometry_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odometry/wheels"));

  // create timer running at 25 Hz
  const unsigned int timer_timeout = 40;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    control_timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &subscription_callback, ON_NEW_DATA));

  // set frame id for transform between parent and child in odometry message
  odometry_msg.header.frame_id = micro_ros_string_utilities_set(odometry_msg.header.frame_id, "/odom");
  odometry_msg.child_frame_id = micro_ros_string_utilities_set(odometry_msg.child_frame_id, "/base_link");

  // set non zero values on diagonals of covariance matrices for velocity and position
  for (size_t i = 0; i < 36; i += 6) {
    odometry_msg.pose.covariance[i] = 0.0001;
    odometry_msg.twist.covariance[i] = 0.0001;
  }

  // synchronize time between uROS and uROS agent
  static time_t timeout_ms = 100;
  static int64_t time_ms = 0;
  while (time_ms <= 0) {
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    time_ms = rmw_uros_epoch_millis(); 
  }
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_publisher_fini(&odometry_publisher, &node));
  RCSOFTCHECK(rcl_timer_fini(&timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}


void setup() {
  // initialise encoders
  encoder_left.set_rotation(0);
  encoder_right.set_rotation(0);

  Serial.begin(115200);
  delay(1000);

  // setup microros transport layer 
  rmw_uros_set_custom_transport(
		true,
		NULL,
		maker_pi_transport_open,
		maker_pi_transport_close,
		maker_pi_transport_write,
		maker_pi_transport_read
	);


  // setup error LED
  pinMode(ERROR_LOOP_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LOOP_LED_PIN, LOW);

  // configure PID
  pid_left_wheel.SetMode(AUTOMATIC);
  pid_right_wheel.SetMode(AUTOMATIC);

  pid_left_wheel.SetSampleTime(1);
  pid_right_wheel.SetSampleTime(1);

  pid_left_wheel.SetOutputLimits(-1.0, 1.0);
  pid_right_wheel.SetOutputLimits(-1.0, 1.0);


  state = WAITING_AGENT;
}


void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}