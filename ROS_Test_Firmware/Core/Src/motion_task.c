/*
 * motion_task.c
 *
 *  Created on: May 27, 2023
 *      Author: sanyi
 */

#include "motion_task.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "queue_msg_types.h"

extern xQueueHandle CommandQueueHandle;
extern xQueueHandle StateQueueHandle;


/**
* @brief Function implementing the MotionTask thread.
* @param argument: Not used
* @retval None
*/
void MotionTaskFunction(void * argument)
{


	CommandMsg *command_msg_ptr;
	command_msg_ptr->Joint_a1 = 3.14;
	command_msg_ptr->Joint_a2 = 0.5;
	command_msg_ptr->Joint_a3 = 0.5;
	StateMsg *state_msg_ptr;

  /* Infinite loop */
  for(;;)
  {
	// read
	  if (xQueueReceive(CommandQueueHandle, command_msg_ptr, 0) == pdPASS)
      {

		  // update
		  state_msg_ptr->Joint_a1_pos = command_msg_ptr->Joint_a1;
		  state_msg_ptr->Joint_a2_pos = command_msg_ptr->Joint_a2;
		  state_msg_ptr->Joint_a3_pos = command_msg_ptr->Joint_a3;
	  }
	// write
	  if (xQueueSend(StateQueueHandle,&state_msg_ptr,portMAX_DELAY) == pdPASS)
	  {

	  }
	// delay
    osDelay(1);
  }
}


