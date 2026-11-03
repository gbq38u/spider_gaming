// Micro-ros thingies
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>

// Messages types
#include <spider_interfaces/srv/spider_speed.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/string.h>

// Servo lib
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Our libs
#include "esp_node.h"
#include "gait.h"
#include "trajectory.h"
#include "kinematics.h"
#include "structs.h"
#include "claw_radio_esp32.h"

#include <stdbool.h>

// Macros
#define CHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { \
    DEBUG_TOPIC_PRINT("BAD THING HAPPENED"); \
    digitalWrite(LED_PIN, HIGH); \
    return false; \
  } \
}

#define CHECKDEBUGPUB(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { \
    digitalWrite(LED_PIN, HIGH); \
    return false; \
  } \
}

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

#define D2R ((float)M_PI / 180.0f)

// Servo params
#define SERVO_COUNT     18
#define SERVO_MIN_US    500
#define SERVO_MAX_US    2400
#define SERVO_RANGE_US  1900.0f
#define SERVO_RANGE_DEG 180.0f

// Pin definitions
#define LED_PIN    2
#define SERVO1_PIN 18
#define SERVO2_PIN 19
#define SERVO3_PIN 21

// Servo objects
Adafruit_PWMServoDriver pca1(0x41);
Adafruit_PWMServoDriver pca2(0x40);
Adafruit_PWMServoDriver* const drivers[] = {&pca1, &pca2};
SemaphoreHandle_t pcaBusMutex = nullptr;
bool claw_radio_ready = false;

bool pca_connection_failed = false;

const servo_port_t servoPorts[SERVO_COUNT] = {
  {1, 15}, {1, 14}, {1, 13}, // J1-J3: left front, 0x40
  {0, 15}, {0, 14}, {0, 13}, // J4-J6: right front, 0x41
  {1, 12}, {1, 11}, {1, 10}, // J7-J9: left middle, 0x40
  {0, 12}, {0, 11}, {0, 10}, // J10-J12: right middle, 0x41
  {1,  9}, {1,  8}, {1,  7}, // J13-J15: left rear, 0x40
  {0,  9}, {0,  8}, {0,  7}  // J16-J18: right rear, 0x41
};

// Our structs
static leg_t legLR = {
    .Servos = {
        .ServoPorts  = { servoPorts[12], servoPorts[13], servoPorts[14] },
        .ServoAngles = {      90.0f,     15.0f,                          15.0f },
    },
    .Geometry = {
        .LinkLengths     = {   3.6f,      4.8f,                           7.2f },
        .JointOffsets    = {   0.0f,      0.0f, (float)M_PI / 2.0f - 0.261799f },
        .BaseTranOffsets = {   0.0f,      0.0f,                           0.0f },
        .BaseRotOffsets  = {   0.0f,      0.0f,                           60.0f * D2R },
        .JointHome       = {           (float)M_PI / 2.0f,
                             15.0f * (float)M_PI / 180.0f,
                             15.0f * (float)M_PI / 180.0f },
        .EFHome          = {   -6.5f,      3.75f,                            -8.0f },
    },
    .Cycle = { 0 },
};

static leg_t legRR = {
    .Servos = {
        .ServoPorts  = { servoPorts[15], servoPorts[16], servoPorts[17] },
        .ServoAngles = {      90.0f,     15.0f,                          15.0f },
    },
    .Geometry = {
        .LinkLengths     = {   3.6f,      4.8f,                           7.2f },
        .JointOffsets    = {   0.0f,      0.0f, (float)M_PI / 2.0f - 0.261799f },
        .BaseTranOffsets = {   0.0f,      0.0f,                           0.0f },
        .BaseRotOffsets  = {   0.0f,      0.0f,                           120.0f * D2R },
        .JointHome       = {           (float)M_PI / 2.0f,
                             15.0f * (float)M_PI / 180.0f,
                             15.0f * (float)M_PI / 180.0f },
        .EFHome          = {   -6.5f,      -3.75f,                           -8.0f },
    },
    .Cycle = { 0 },
};

static leg_t legRM = {
    .Servos = {
        .ServoPorts  = { servoPorts[9], servoPorts[10], servoPorts[11] },
        .ServoAngles = {      90.0f,     15.0f,                          15.0f },
    },
    .Geometry = {
        .LinkLengths     = {   3.6f,      4.8f,                           7.2f },
        .JointOffsets    = {   0.0f,      0.0f, (float)M_PI / 2.0f - 0.261799f },
        .BaseTranOffsets = {   0.0f,      0.0f,                           0.0f },
        .BaseRotOffsets  = {   0.0f,      0.0f,                           180.0f * D2R },
        .JointHome       = {           (float)M_PI / 2.0f,
                             15.0f * (float)M_PI / 180.0f,
                             15.0f * (float)M_PI / 180.0f },
        .EFHome          = {   0.0f,      -7.5f,                           -8.0f },
    },
    .Cycle = { 0 },
};

static leg_t legLM = {
    .Servos = {
        .ServoPorts  = { servoPorts[6], servoPorts[7], servoPorts[8] },
        .ServoAngles = {      90.0f,     15.0f,                          15.0f },
    },
    .Geometry = {
        .LinkLengths     = {   3.6f,      4.8f,                           7.2f },
        .JointOffsets    = {   0.0f,      0.0f, (float)M_PI / 2.0f - 0.261799f },
        .BaseTranOffsets = {   0.0f,      0.0f,                           0.0f },
        .BaseRotOffsets  = {   0.0f,      0.0f,                           0.0f },
        .JointHome       = {           (float)M_PI / 2.0f,
                             15.0f * (float)M_PI / 180.0f,
                             15.0f * (float)M_PI / 180.0f },
        .EFHome          = {   0.0f,      7.5f,                          -8.0f },
    },
    .Cycle = { 0 },
};


static leg_t legLF = {
    .Servos = {
        .ServoPorts  = { servoPorts[0], servoPorts[1], servoPorts[2] },
        .ServoAngles = {      90.0f,     15.0f,                          15.0f },
    },
    .Geometry = {
        .LinkLengths     = {   3.6f,      4.8f,                           7.2f },
        .JointOffsets    = {   0.0f,      0.0f, (float)M_PI / 2.0f - 0.261799f },
        .BaseTranOffsets = {   0.0f,      0.0f,                           0.0f },
        .BaseRotOffsets  = {   0.0f,      0.0f,                           300.0f * D2R },
        .JointHome       = {           (float)M_PI / 2.0f,
                             15.0f * (float)M_PI / 180.0f,
                             15.0f * (float)M_PI / 180.0f },
        .EFHome          = {   6.5f,      3.75f,                            -8.0f },
    },
    .Cycle = { 0 },
};

static leg_t legRF = {
    .Servos = {
        .ServoPorts  = { servoPorts[3], servoPorts[4], servoPorts[5] },
        .ServoAngles = {      90.0f,     15.0f,                          15.0f },
    },
    .Geometry = {
        .LinkLengths     = {   3.6f,      4.8f,                           7.2f },
        .JointOffsets    = {   0.0f,      0.0f, (float)M_PI / 2.0f - 0.261799f },
        .BaseTranOffsets = {   0.0f,      0.0f,                           0.0f },
        .BaseRotOffsets  = {   0.0f,      0.0f,                           240.0f * D2R },
        .JointHome       = {           (float)M_PI / 2.0f,
                             15.0f * (float)M_PI / 180.0f,
                             15.0f * (float)M_PI / 180.0f },
        .EFHome          = {   6.5f,      -3.75f,                            -8.0f },
    },
    .Cycle = { 0 },
};



static spider_t spider = {
  .Legs = { &legLF, &legRF, &legLM, &legRM, &legLR, &legRR }
};


// Messages
std_msgs__msg__String err_msg;
#define err_buf_cap 64
static char debug_msg_storage[err_buf_cap] = {0}; 

std_msgs__msg__Int16MultiArray servo_msg;
static int16_t servo_msg_storage[3] = {90, 15, 15};

spider_interfaces__srv__SpiderSpeed_Request  speed_req_msg;
spider_interfaces__srv__SpiderSpeed_Response speed_res_msg;
static float     speed_vals[2] = {0.0, 0.0};
static int16_t   speed_res = 0;

static float servo_angles[3] = {90.0, 15.0, 15.0};

// ROS objects
rcl_timer_t          motion_timer;
rcl_timer_t          servo_timer;
rcl_publisher_t      debug_pub;
const char *         debug_topic_name     = "/esp/debug";
rcl_subscription_t   servo_sub;
const char *         servo_topic_name     = "/esp/servos";
rcl_service_t        speed_srv;
const char *         speed_srv_name       = "/esp/speed";

rclc_support_t     support;
rcl_init_options_t init_options;
rcl_node_t         node;
rclc_executor_t    executor;
rcl_allocator_t    allocator;

// State machine
typedef enum {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} states_t;
states_t state = WAITING_AGENT;

// helper functions
void leg_write_servo_angles(leg_t * leg)
{
    const int us[3] = {
        SERVO_MIN_US + (int)(leg->Servos.ServoAngles[0] * (SERVO_RANGE_US / SERVO_RANGE_DEG)),
        SERVO_MIN_US + (int)(leg->Servos.ServoAngles[1] * (SERVO_RANGE_US / SERVO_RANGE_DEG)),
        SERVO_MIN_US + (int)(leg->Servos.ServoAngles[2] * (SERVO_RANGE_US / SERVO_RANGE_DEG)),
    };

    // Радио управляет другими каналами тех же PCA из отдельной задачи.
    if (pcaBusMutex && xSemaphoreTake(pcaBusMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
    for (int j = 0; j < 3; j++) {
        const uint8_t drv = leg->Servos.ServoPorts[j].driver;
        const uint8_t ch  = leg->Servos.ServoPorts[j].channel;
        drivers[drv]->writeMicroseconds(ch, us[j]);
    }
    if (pcaBusMutex) xSemaphoreGive(pcaBusMutex);
}

void leg_change_servo_angles(leg_t * leg, float a1, float a2, float a3)
{
    leg_set_servo_angles(leg, a1, a2, a3);
    leg_write_servo_angles(leg);
}

void spider_homecoming(spider_t * spider)
{
    spider_set_home(spider);
    for (int i = 0; i < 6; i++) {
        leg_write_servo_angles(spider->Legs[i]);
    }
}

// Debug

static char debug_buf[err_buf_cap];

bool DEBUG_TOPIC_PRINT(const char * msgout)
{
  size_t len = strnlen(msgout, err_buf_cap - 1);
  memcpy(debug_buf, msgout, len);
  debug_buf[len] = '\0';
  err_msg.data.data = debug_buf;
  err_msg.data.size = len;
  return rcl_publish(&debug_pub, &err_msg, NULL) == RCL_RET_OK;
}

// bool DEBUG_TOPIC_PRINT(const char * msgout)
// {
//   err_msg.data.data = (char*)msgout;
//   int len = strlen(msgout);
//   err_msg.data.size =  (len < err_buf_cap) ? (len) : (err_buf_cap - 1);
//   CHECKDEBUGPUB(rcl_publish(&debug_pub, &err_msg, NULL));
//   return true;
// }

void motion_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;

    if (gait_consume_cmd_dirty()) {
        gait_regenerate_all(&spider);
    }

    for (int i = 0; i < 6; i++) {
        gait_motion_tick(spider.Legs[i]);
    }
}

void servo_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;

    for (int i = 0; i < 6; i++) {
        leg_t * leg = spider.Legs[i];
        gait_servo_tick(leg);
        leg_write_servo_angles(leg);
    }
}


void speed_callback(const void * req_msgin, void * res_msgin)
{
  DEBUG_TOPIC_PRINT("SPEED CALLBACK");

  const spider_interfaces__srv__SpiderSpeed_Request * req_msg = (const spider_interfaces__srv__SpiderSpeed_Request *)req_msgin;
  spider_interfaces__srv__SpiderSpeed_Response      * res_msg  = (spider_interfaces__srv__SpiderSpeed_Response *)     res_msgin;

  if (!isfinite((float)req_msg->data[0]) || !isfinite((float)req_msg->data[1])) { return; }

  speed_vals[0] = (float)req_msg->data[0];
  speed_vals[1] = (float)req_msg->data[1];
  gait_set_command((float)req_msg->data[0], (float)req_msg->data[1]);

  res_msg->err = 0;
}

void servo_callback(const void *msgin)
{
  DEBUG_TOPIC_PRINT("SERVO CALLBACK");

  const std_msgs__msg__Int16MultiArray *msg =
      (const std_msgs__msg__Int16MultiArray *)msgin;

  if (msg->data.size != 3) {
    DEBUG_TOPIC_PRINT("YOU'VE MESSED SERVO MSG");
    return;
  }

  /* Route to whichever leg this is meant for. For now, hardwire to
   * the left-front leg — you'll parameterize this later. */
  leg_t * leg = &legLF;

  leg->Servos.ServoAngles[0] = constrainf((float)msg->data.data[0], 0.0f, 180.0f);
  leg->Servos.ServoAngles[1] = constrainf((float)msg->data.data[1], 0.0f, 180.0f);
  leg->Servos.ServoAngles[2] = constrainf((float)msg->data.data[2], 0.0f, 180.0f);

  leg_write_servo_angles(leg);
}

// Create entities
bool create_entities()
{
  digitalWrite(LED_PIN, LOW);

  const char * node_name = "esp_node";
  const char * ns        = "";
  const int    domain_id = 0;

  allocator    = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  CHECKDEBUGPUB(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  CHECKDEBUGPUB(rclc_node_init_default(&node, node_name, ns, &support));

  // Debug
  CHECKDEBUGPUB(rclc_publisher_init_best_effort(
    &debug_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    debug_topic_name
  ));

  // Timer
  const unsigned int motion_timer_period_ms = 20;
  CHECK(rclc_timer_init_default(
    &motion_timer,
    &support,
    RCL_MS_TO_NS(motion_timer_period_ms),
    motion_timer_callback
  ));

  const unsigned int servo_timer_period_ms = 20;
  CHECK(rclc_timer_init_default(
    &servo_timer,
    &support,
    RCL_MS_TO_NS(servo_timer_period_ms),
    servo_timer_callback
  ));

  // Subs
  CHECK(rclc_subscription_init_default(
    &servo_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    servo_topic_name
  ));

  // Services
  CHECK(rclc_service_init_default(
    &speed_srv,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(spider_interfaces, srv, SpiderSpeed),
    speed_srv_name
  ));

  unsigned int num_handles = 6;
  executor = rclc_executor_get_zero_initialized_executor();
  CHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  CHECK(rclc_executor_add_timer       (&executor, &motion_timer));
  CHECK(rclc_executor_add_timer       (&executor, &servo_timer));
  CHECK(rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA));
  CHECK(rclc_executor_add_service     (&executor, &speed_srv, &speed_req_msg, &speed_res_msg, &speed_callback));

  return true;
}

// Destroy entities
void destroy_entities() 
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  
  rcl_timer_fini       (&motion_timer);
  rcl_timer_fini       (&servo_timer);
  rcl_subscription_fini(&servo_sub, &node);
  rcl_publisher_fini   (&debug_pub, &node);
  rcl_service_fini     (&speed_srv, &node);

  rclc_executor_fini   (&executor);

  rcl_init_options_fini(&init_options);
  rcl_node_fini        (&node);
  rclc_support_fini    (&support);
}

void setup() {
  set_microros_transports();

  delay(100);

  Wire.begin(21,22);
  Wire.setClock(400000);

  const bool found1 = pca1.begin();
  if (found1)
  {
    for (uint8_t ch = 0; ch < 16; ++ch) pca1.setPWM(ch, 0, 0);
  }
  const bool found2 = pca2.begin();
  if (found2)
  {
    for (uint8_t ch = 0; ch < 16; ++ch) pca2.setPWM(ch, 0, 0);
  }

  pca_connection_failed = !(found1 && found2);

  pca1.setPWMFreq(50);
  pca2.setPWMFreq(50);

  delay(100);
  pcaBusMutex = xSemaphoreCreateMutex();

  spider_homecoming(&spider);

  /* Tripod gait. Groups alternate, so every group has a foot on each side. */
  static const int tripod_phase[6] = { 0, 5, 5, 0, 0, 5 };
  /* indices:                          LF RF LM RM LR RR */

  const int steps_per_cycle = SWING_STEPS + STANCE_STEPS;   /* 4 */
  for (int i = 0; i < 6; i++) {
      gait_init(spider.Legs[i], tripod_phase[i] % steps_per_cycle);
  }

  err_msg.data.capacity = err_buf_cap;
  err_msg.data.size     = 0;
  err_msg.data.data     = debug_msg_storage;

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (!found2){digitalWrite(LED_PIN, HIGH);}

  // pca2 имеет адрес 0x40 (левая клешня), pca1 — 0x41 (правая).
  // Отдельная задача радио продолжает работать во время ожидания micro-ROS.
  if (!pca_connection_failed && pcaBusMutex) {
    claw_radio_ready = ClawRadio::begin(pca2, pca1, pcaBusMutex);
  }

  state = WAITING_AGENT;
  delay(100);
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
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
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
