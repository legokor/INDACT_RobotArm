/*
 * QueueMsgTypes.h
 *
 *  Created on: Jun 2, 2023
 *      Author: sanyi
 */

#ifndef INC_QUEUE_MSG_TYPES_H_
#define INC_QUEUE_MSG_TYPES_H_

typedef struct
{
	double Joint_a1;
	double Joint_a2;
	double Joint_a3;
} CommandMsg;

typedef struct
{
	double Joint_a1_pos;
	double Joint_a2_pos;
	double Joint_a3_pos;
} StateMsg;


#endif /* INC_QUEUE_MSG_TYPES_H_ */
