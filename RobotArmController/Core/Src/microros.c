/**
 * @brief Micro-ROS related variables and functions.
 *
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2024-10-14
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include "logger.h"
#include "rtos_priorities.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MICROROS_TASK_STACK_SIZE 5120
#define MICROROS_TASK_PRIORITY TASK_PRIORITY_BELOW_NORMAL

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static StaticTask_t microrosTaskBuffer;
static StackType_t microrosTaskStack[MICROROS_TASK_STACK_SIZE];
TaskHandle_t microrosTaskHandle = NULL;

rcl_allocator_t allocator = {0};
rclc_support_t support = {0};

rcl_node_t node;
rcl_subscription_t command_subscription = {0};
rcl_publisher_t state_publisher = {0};
rcl_timer_t state_timer = {0};

std_msgs__msg__Int32 command_msg = {0};

int32_t counter = 0;

/* Private function prototypes -----------------------------------------------*/
extern bool cubemx_transport_open(struct uxrCustomTransport *transport);
extern bool cubemx_transport_close(struct uxrCustomTransport *transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
extern size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

extern void *microros_allocate(size_t size, void *state);
extern void microros_deallocate(void *pointer, void *state);
extern void *microros_reallocate(void *pointer, size_t size, void *state);
extern void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

static void microrosTask(void *pvParameters);

static void init_microros(void);
static int init_controller_node(rclc_executor_t *executor);

static void command_subscription_callback(const void *msgin);
static void state_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

/* Function definitions ----------------------------------------------------- */

void setup_microros_rtos(void)
{
    microrosTaskHandle = xTaskCreateStatic(
        microrosTask,
        "microros",
        MICROROS_TASK_STACK_SIZE,
        NULL,
        MICROROS_TASK_PRIORITY,
        microrosTaskStack,
        &microrosTaskBuffer);
    configASSERT(microrosTaskHandle != NULL);
}

static void microrosTask(void *pvParameters)
{
    init_microros();

    // PING ------------------------- */
    logInfo("Start waiting for micro-ros-agent by PING...");
    unsigned int attempts = 1;
    while (rmw_uros_ping_agent(50, 1) != RCL_RET_OK)
    {
        logWarn("Waiting for micro-ros-agent... (Attempt %u", attempts++);
        vTaskDelay(pdMS_TO_TICKS(950));
    }
    logInfo("micro-ros-agent ready! Total attempts: %u.", attempts - 1);
    // PING ------------------------- */

    rcl_ret_t rc = RCL_RET_OK;

    allocator = rcl_get_default_allocator();

    rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK)
    {
        logError("RCL support initialization failed! Error code: %d.", rc);
        vTaskSuspend(NULL);
    }

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    const size_t number_of_handles = 2;
    rc = rclc_executor_init(&executor, &support.context, number_of_handles, &allocator);
    if (rc != RCL_RET_OK)
    {
        logError("RCLC executor initialization failed! Error code: %d.", rc);
        vTaskSuspend(NULL);
    }

    int ret = init_controller_node(&executor);
    if (ret != 0)
    {
        logError("Controller node initialization failed! Error code: %d.", ret);
        vTaskSuspend(NULL);
    }

    // spin()
    for (;;)
    {
        rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        if ((rc != RCL_RET_OK) && (rc != RCL_RET_TIMEOUT))
        {
            logError("RCLC executor spin failed!");
            vTaskSuspend(NULL);
        }

        // Yield
        vTaskDelay(10);
    }
}

void init_microros(void)
{
    rmw_uros_set_custom_transport(
            true,
            NULL,
            cubemx_transport_open,
            cubemx_transport_close,
            cubemx_transport_write,
            cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator))
    {
        printf("Error on default allocators!\n");
    }
}

int init_controller_node(rclc_executor_t *executor)
{
    rcl_ret_t rc = RCL_RET_OK;
    rc = rclc_node_init_default(&node, "controller_node", "", &support);
    if (rc != RCL_RET_OK)
    {
        logError("RCLC node init failed. Error code: %d", rc);
        return 1;
    }

    // Command subscription
    const char *command_topic = "/set_counter";
    const rosidl_message_type_support_t *command_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
    rc = rclc_subscription_init_default(&command_subscription, &node, command_type_support, command_topic);
    if (rc != RCL_RET_OK)
    {
        logError("RCLC subscription (topic %s) init failed. Error code: %d", command_topic, rc);
        return 2;
    }

    // State publisher
    const char *state_topic = "/counter";
    const rosidl_message_type_support_t *state_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
    rc = rclc_publisher_init_default(&state_publisher, &node, state_type_support, state_topic);
    if (rc != RCL_RET_OK)
    {
        logError("RCLC publisher (topic %s) init failed. Error code: %d", state_topic, rc);
        return 3;
    }

    // State timer
    const unsigned int timer_period = RCL_MS_TO_NS(1000);
    rc = rclc_timer_init_default(&state_timer, &support, timer_period, state_timer_callback);
    if (rc != RCL_RET_OK)
    {
        logError("RCLC timer init failed. Error code: %d", rc);
        return 4;
    }

    // Add handles to executor
    rc = rclc_executor_add_subscription(executor, &command_subscription, &command_msg, &command_subscription_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        logError("RCLC executor add subscription failed. Error code: %d", rc);
        return 5;
    }
    rc = rclc_executor_add_timer(executor, &state_timer);
    if (rc != RCL_RET_OK)
    {
        logError("RCLC executor add timer failed. Error code: %d", rc);
        return 6;
    }

    return 0;
}

void command_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    counter = msg->data;
}

void state_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer == NULL)
    {
        return;
    }

    std_msgs__msg__Int32 msg;
    msg.data = counter;
    rcl_ret_t rc = rcl_publish(&state_publisher, &msg, NULL);
    if (rc != RCL_RET_OK)
    {
        logWarn("RCL publish failed.");
    }

    counter++;
}
