/*
 * microros_task.c
 *
 *  Created on: May 27, 2023
 *      Author: sanyi
 */

#include "microros_task.h"
#include "usb_device.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "queue_msg_types.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>

#include <trajectory_msgs/msg/joint_trajectory.h>
#include <sensor_msgs/msg/joint_state.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

extern USBD_HandleTypeDef hUsbDeviceFS;
extern void MX_USB_DEVICE_Init(void);
extern xQueueHandle CommandQueueHandle;
extern xQueueHandle StateQueueHandle;

rcl_publisher_t control_pub;
sensor_msgs__msg__JointState control_pub_msg;

CommandMsg *command_msg_ptr;
StateMsg *state_msg_ptr;

char* JointNames[] = {"joint_a1", "joint_a2", "joint_a3"};

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


void control_sub_callback(const void * msgin)
{
  // Cast received message to used type
  const trajectory_msgs__msg__JointTrajectory * msg = (const trajectory_msgs__msg__JointTrajectory *)msgin;

  // Process message
  for (size_t i = 0; i < msg->joint_names.size; ++i) {
	  if(msg->joint_names.data[i].data == JointNames[0])
	  {
		  command_msg_ptr->Joint_a1 = msg->points.data->positions.data[i];
	  }
	  if(msg->joint_names.data[i].data == JointNames[1])
	  {
		  command_msg_ptr->Joint_a2 = msg->points.data->positions.data[i];
	  }
	  if(msg->joint_names.data[i].data == JointNames[2])
	  {
		  command_msg_ptr->Joint_a3 = msg->points.data->positions.data[i];
	  }
  }
  if (xQueueSend(CommandQueueHandle,&command_msg_ptr,portMAX_DELAY) == pdPASS){

  }

}

void pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {

		// load data
		if (xQueueReceive(StateQueueHandle, state_msg_ptr, 0) == pdPASS)
		{
			control_pub_msg.position.data[0] = state_msg_ptr->Joint_a1_pos;
			control_pub_msg.position.data[1] = state_msg_ptr->Joint_a2_pos;
			control_pub_msg.position.data[2] = state_msg_ptr->Joint_a3_pos;

		}

		RCSOFTCHECK(rcl_publish(&control_pub, &control_pub_msg, NULL));
	}

}


/**
* @brief Function implementing the MicrorosTask thread.
* @param argument: Not used
* @retval None
*/
void MicrorosTaskFunction(void * argument)
{
    /* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	// micro-ROS configuration

	rmw_uros_set_custom_transport(
	  true,
	  (void *) &hUsbDeviceFS,
	  cubemx_transport_open,
	  cubemx_transport_close,
	  cubemx_transport_write,
	  cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}

	command_msg_ptr = pvPortMalloc(sizeof (CommandMsg));
	state_msg_ptr = pvPortMalloc(sizeof (StateMsg));

	// micro-ROS app
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rcl_subscription_t subscriber;
	rcl_timer_t pub_timer;
	rclc_support_t support;
	rcl_node_t node;
	rcl_ret_t rc;

	const char * sub_topic_name = "command_interface";
	const char * pub_topic_name = "state_broadcaster";
	const char * node_name = "control_node";
	const char * node_namespace = "microros";
	const unsigned int pub_timer_timeout = 100; // in ms

	trajectory_msgs__msg__JointTrajectory sub_msg;

	const rosidl_message_type_support_t * sub_type_support =
		ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs,msg,JointTrajectory);

	const rosidl_message_type_support_t * pub_type_support =
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,JointState);
	allocator = rcl_get_default_allocator();

	//create init_options
	RCCHECK(rc = rclc_support_init(&support, 0, NULL, &allocator));


	// create node
	RCCHECK(rc = rclc_node_init_default(&node, node_name, node_namespace, &support));

	// create publisher
	RCCHECK(rc = rclc_publisher_init_default(&control_pub, &node, pub_type_support, pub_topic_name));


	// create timer for publisher
	RCCHECK(rc = rclc_timer_init_default(&pub_timer, &support, RCL_MS_TO_NS(pub_timer_timeout), pub_timer_callback));


	// create subscriber
	RCCHECK(rc = rclc_subscription_init_default(&subscriber, &node, sub_type_support, sub_topic_name));

	// create executer
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &pub_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &control_sub_callback, ON_NEW_DATA));

	// todo: Add pub data init
	static micro_ros_utilities_memory_conf_t conf = {0};
	micro_ros_utilities_create_message_memory(sub_type_support, &sub_msg, conf);
	micro_ros_utilities_create_message_memory(pub_type_support, &control_pub_msg, conf);

	rclc_executor_spin(&executor);


	RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_timer_fini(&pub_timer));
	RCCHECK(rcl_publisher_fini(&control_pub, &node));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rclc_support_fini(&support));

	micro_ros_utilities_destroy_message_memory(sub_type_support, &sub_msg, conf);
	micro_ros_utilities_destroy_message_memory(pub_type_support, &control_pub_msg, conf);

	vTaskDelete(NULL);

	/* Infinite loop */
	for(;;)
	{
	  osDelay(1);
	}
}

