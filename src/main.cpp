#include <Arduino.h>

// standard libraries
#include <stdio.h>
#include <time.h>
#include <math.h>

// RP2040 multicore libraries
extern "C"
{
#include <pico/multicore.h>
#include <pico_sync/include/pico/mutex.h>
}

// microros dependencies
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/env.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

// microros message definitions
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>

// external libraries
#include <pio_rotary_encoder.h>

// custom transport layer definition
#include "maker_pi_transport.h"
#include "pwm_conf.h"
#include "pid.h"
#include "utils.h"

#define ERROR_LOOP_LED_PIN (16)
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
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

// error loop used to handle exceptions of uROS
void error_loop()
{
  Serial.println("Error occured");
  while (1)
  {
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

// Stuctures used to pass messages between CPU cores
typedef struct
{
  double position;
  double velocity;
  double effort;
} joint_state_t;

volatile joint_state_t left_wheel_state;
volatile joint_state_t right_wheel_state;
volatile double left_wheel_setpoint;
volatile double right_wheel_setpoint;

mutex_t msg_sync_mutex;

// Stuct holding robot position used to compute odometry
typedef struct
{
  double x;
  double y;
  double theta;
} robot_state_t;

robot_state_t robot_state;

// Required for time sync
extern int clock_gettime(clockid_t unused, struct timespec *tp);

// Create handles for RCLC objects
rclc_support_t support;
rcl_node_t node;
rcl_timer_t publish_timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t odometry_publisher;
rcl_publisher_t joint_state_publisher;
rcl_publisher_t imu_publisher;
nav_msgs__msg__Odometry odometry_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__JointState joint_state_msg;

// Init values for uROS connection sm
bool micro_ros_init_successful;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  double lin_x = msg->linear.x;
  double ang_z = msg->angular.z;

  double vr = (ang_z * (-wheel_separation / 2.0) + lin_x) / M_PI / wheel_radius;
  double vl = (ang_z * (wheel_separation / 2.0) + lin_x) / M_PI / wheel_radius;

  // Pass setpoints via thread-save mutex
  if (mutex_try_enter(&msg_sync_mutex, NULL))
  {
    left_wheel_setpoint = vr;
    right_wheel_setpoint = vl;
    mutex_exit(&msg_sync_mutex);
  }
}

void publisher_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // update time stamp
    struct timespec tv = {0};
    clock_gettime(0, &tv);
    odometry_msg.header.stamp.nanosec = tv.tv_nsec;
    odometry_msg.header.stamp.sec = tv.tv_sec;

    joint_state_msg.header.stamp.nanosec = tv.tv_nsec;
    joint_state_msg.header.stamp.sec = tv.tv_sec;

    // Get data from PID controller via thread-save mutex
    if (mutex_try_enter(&msg_sync_mutex, NULL))
    {
      joint_state_msg.position.data[0] = wrap_angle(left_wheel_state.position);
      joint_state_msg.velocity.data[0] = left_wheel_state.velocity;
      joint_state_msg.effort.data[0] = left_wheel_state.effort;

      joint_state_msg.position.data[1] = wrap_angle(right_wheel_state.position);
      joint_state_msg.velocity.data[1] = right_wheel_state.velocity;
      joint_state_msg.effort.data[1] = right_wheel_state.effort;
      mutex_exit(&msg_sync_mutex);
    }

    double dt = 0.01;

    double vl = joint_state_msg.velocity.data[0];
    double vr = joint_state_msg.velocity.data[1];
    double omega = (vr - vl) / 2.0;
    double lin_vel = (vl + vr) / 2.0;

    odometry_msg.twist.twist.linear.x = lin_vel * cos(robot_state.theta);
    odometry_msg.twist.twist.linear.y = lin_vel * sin(robot_state.theta);
    robot_state.x += odometry_msg.twist.twist.linear.x * dt;
    robot_state.y += odometry_msg.twist.twist.linear.y * dt;

    robot_state.theta += omega * dt;
    odometry_msg.twist.twist.angular.z = omega;

    odometry_msg.pose.pose.position.x = robot_state.x;
    odometry_msg.pose.pose.position.y = robot_state.y;

    double q[4];
    euler_to_quat(0.0, 0.0, robot_state.theta, q);
    odometry_msg.pose.pose.orientation.w = q[0];
    odometry_msg.pose.pose.orientation.x = q[1];
    odometry_msg.pose.pose.orientation.y = q[2];
    odometry_msg.pose.pose.orientation.z = q[3];

    // publish odometry and joint state messages
    RCSOFTCHECK(rcl_publish(&odometry_publisher, &odometry_msg, NULL));
    RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "muter_hardware_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &odometry_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "muter/odom"));

  // create /joint_statestopic publisher
  RCCHECK(rclc_publisher_init_default(
      &joint_state_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/joint_states"));

  // create timer running at 10 Hz
  const unsigned int publish_timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
      &publish_timer,
      &support,
      RCL_MS_TO_NS(publish_timer_timeout),
      publisher_timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &subscription_callback, ON_NEW_DATA));

  // set frame id for transform between parent and child in odometry message
  odometry_msg.header.frame_id = micro_ros_string_utilities_init("odom");
  odometry_msg.child_frame_id = micro_ros_string_utilities_init("base_link");

  // Initialise /joint_statesmessage
  joint_state_msg.header.frame_id = micro_ros_string_utilities_init("base_link");
  joint_state_msg.name.data = (rosidl_runtime_c__String *)malloc(3 * sizeof(rosidl_runtime_c__String));
  joint_state_msg.name.size = 2;
  joint_state_msg.name.capacity = 2;
  joint_state_msg.name.data[0] = micro_ros_string_utilities_init("left_wheel");
  joint_state_msg.name.data[1] = micro_ros_string_utilities_init("right_wheel");

  joint_state_msg.position.data = (double *)malloc(2 * sizeof(double));
  joint_state_msg.position.size = 2;
  joint_state_msg.position.capacity = 2;

  joint_state_msg.velocity.data = (double *)malloc(2 * sizeof(double));
  joint_state_msg.velocity.size = 2;
  joint_state_msg.velocity.capacity = 2;

  joint_state_msg.effort.data = (double *)malloc(2 * sizeof(double));
  joint_state_msg.effort.size = 2;
  joint_state_msg.effort.capacity = 2;
  joint_state_msg.effort.data[0] = 0.0;
  joint_state_msg.effort.data[1] = 0.0;

  // set non zero values on diagonals of covariance matrices for velocity and position
  for (size_t i = 0; i < 36; i += 6)
  {
    odometry_msg.pose.covariance[i] = 0.0001;
    odometry_msg.twist.covariance[i] = 0.0001;
  }

  // synchronize time between uROS and uROS agent
  static time_t timeout_ms = 100;
  static int64_t time_ms = 0;
  while (time_ms <= 0)
  {
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    time_ms = rmw_uros_epoch_millis();
  }
  Serial.println("Node initialised");
  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  micro_ros_string_utilities_destroy(&joint_state_msg.header.frame_id);
  micro_ros_string_utilities_destroy(&odometry_msg.header.frame_id);
  micro_ros_string_utilities_destroy(&odometry_msg.child_frame_id);

  RCSOFTCHECK(rcl_publisher_fini(&odometry_publisher, &node));
  RCSOFTCHECK(rcl_publisher_fini(&joint_state_publisher, &node));
  RCSOFTCHECK(rcl_timer_fini(&publish_timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

inline void set_control(uint pin_forward, uint pin_backward, double value)
{
  if (value > 0.0)
  {
    pwm_set_gpio_level(pin_forward, uint16_t(value * double(MAX_PWM_VAL)));
  }
  else
  {
    pwm_set_gpio_level(pin_backward, uint16_t(-value * double(MAX_PWM_VAL)));
  }
}

void core1_entry()
{
  // 25 Hz
  const uint64_t loop_time_us = 40000;

  // define PID for wheels
  PID_stat_t left_wheel_pid_stats = {
      .kp = 0.0,
      .ki = 0.4,
      .kd = 0.0,
      .out_min = -1.0,
      .out_max = 1.0,
      .dt = US_TO_S_D(loop_time_us)};

  PID_stat_t right_wheel_pid_stats = {
      .kp = 0.0,
      .ki = 0.4,
      .kd = 0.0,
      .out_min = -1.0,
      .out_max = 1.0,
      .dt = US_TO_S_D(loop_time_us)};

  // Init state structures
  joint_state_t local_left_wheel_state = {0};
  joint_state_t local_right_wheel_state = {0};
  double local_left_wheel_setpoint = 0.0;
  double local_right_wheel_setpoint = 0.0;
  double last_position_left = 0.0;
  double last_position_right = 0.0;

  // initialise encoders
  encoder_left.set_rotation(0);
  encoder_right.set_rotation(0);

  absolute_time_t start_time;
  absolute_time_t sleep_until_time;

  // Setup PWM
  setup_pin_pwm(10);
  setup_pin_pwm(11);
  setup_pin_pwm(8);
  setup_pin_pwm(9);

  while (true)
  {
    start_time = get_absolute_time();
    sleep_until_time = delayed_by_us(start_time, loop_time_us);

    // Non blocking acquire of mutex
    if (mutex_try_enter(&msg_sync_mutex, NULL))
    {
      local_left_wheel_setpoint = left_wheel_setpoint;
      local_right_wheel_setpoint = right_wheel_setpoint;
      mutex_exit(&msg_sync_mutex);
    }

    // update encode readings
    last_position_left = local_left_wheel_state.position;
    last_position_right = local_right_wheel_state.position;
    local_left_wheel_state.position = tic_to_rad(encoder_left.get_rotation());
    local_right_wheel_state.position = -tic_to_rad(encoder_right.get_rotation());

    local_left_wheel_state.velocity = (local_left_wheel_state.position - last_position_left) / US_TO_S_D(loop_time_us);
    double left_wheel_error = local_left_wheel_state.velocity - local_left_wheel_setpoint;
    double left_wheel_control = pid_compute(&left_wheel_pid_stats, left_wheel_error);
    set_control(11, 10, left_wheel_control);

    local_right_wheel_state.velocity = (local_right_wheel_state.position - last_position_right) / US_TO_S_D(loop_time_us);
    double right_wheel_error = local_right_wheel_state.velocity - local_right_wheel_setpoint;
    double right_wheel_control = pid_compute(&right_wheel_pid_stats, right_wheel_error);
    set_control(8, 9, right_wheel_control);

    // Non blocking acquire of mutex
    if (mutex_try_enter(&msg_sync_mutex, NULL))
    {
      left_wheel_state.position = local_left_wheel_state.position;
      left_wheel_state.velocity = local_left_wheel_state.velocity;
      left_wheel_state.effort = NAN;

      right_wheel_state.position = local_right_wheel_state.position;
      right_wheel_state.velocity = local_right_wheel_state.velocity;
      right_wheel_state.effort = NAN;
      mutex_exit(&msg_sync_mutex);
    }

    sleep_until(sleep_until_time);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  mutex_init(&msg_sync_mutex);

  // init odometry
  robot_state.x = 0.0;
  robot_state.y = 0.0;
  robot_state.theta = 0.0;

  // setup microros transport layer
  rmw_uros_set_custom_transport(
      true,
      NULL,
      maker_pi_transport_open,
      maker_pi_transport_close,
      maker_pi_transport_write,
      maker_pi_transport_read);

  // setup error LED
  pinMode(ERROR_LOOP_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LOOP_LED_PIN, LOW);

  // Launch Core 1 of rp2040
  multicore_launch_core1(core1_entry);

  state = WAITING_AGENT;
  Serial.println("Init passed");
}

void loop()
{
  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    Serial.println("Waiting for the agent...");
    break;
  case AGENT_AVAILABLE:
    Serial.println("Attempting to create node");
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    break;
  case AGENT_DISCONNECTED:
    Serial.println("Agent disconnected");
    destroy_entities();
    state = WAITING_AGENT;
    break;
  default:
    break;
  }
}