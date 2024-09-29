/**
 * @file KAR_GEO_interface.h
 *
 * ""
 *
 * $Author: Erdei Sándor
 * $Date: 2023-07-28
 * $Revision: $
 *
 * @copyright LEGO kör
 */

/*# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
 #																												#
 #		Detailed description																					#
 #																												#
 # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # */

#ifndef KAR_GEO_INTERFACE_H
#define KAR_GEO_INTERFACE_H

/****************************************************************************************************************
 * 																												*
 * Includes																										*
 * 																												*
 *****************************************************************************************************************/

#include <stdbool.h>

#include "stm32f7xx_hal.h"

/****************************************************************************************************************
 * 																												*
 * Public Types																									*
 * 																												*
 *****************************************************************************************************************/

/**
 * The robot has 6 limit switches which serves a safety function. To prevent incidents the switches trigger interrupts when switched on,
 * which stops the appropriate motor.
 *
 * FI axis' maximum point: when the arm is rotated all the way clockwise
 * FI axis' zero point: when the arm is rotated all the way counter-clockwise
 * Z axis' maximum point: lower sitch on the Z axis
 * Z axis' zero point: upper switch on the Z axis
 * R axis' maximum point: when the arm reaches the furthest
 * R axis' zero ponit: when the arm is fully retracted
 */
typedef struct
{
	bool max_point;
	bool null_point;
} s_GEO_LimitSwitch;

/*
 * The arm moves in a cylindrical coordinate system.
 * To define points in this space requires three coordinates:
 *  - angular coordinate - fi
 *  - height - z
 *  - radial distance from origin - r
 */
typedef struct
{
	int32_t fi;
	int32_t z;
	int32_t r;
} s_GEO_ToolPosition_Cylinder;

typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
} s_GEO_ToolPosition_Descartes;

#endif		// KAR_GEO_INTERFACE_H

/* End of file KAR_GEO_interface.h */
