/**
 * @file KAR_GEO_InverseGeometry_handler.h
 *
 * "Makes coordinate-system changing calculations"
 *
 * $Author: Erdei Sándor
 * $Date: 2023-08-09
 * $Revision: $
 *
 * @copyright LEGO kör - INDACT project
 */

/*# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
 #																												#
 #		* Detailed description *																				#
 #																												#
 # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # */

#ifndef KAR_GEO_INVERSE_GEOMETRY_HANDLER_H
#define KAR_GEO_INVERSE_GEOMETRY_HANDLER_H

/****************************************************************************************************************
 * 																												*
 * Includes																										*
 * 																												*
 *****************************************************************************************************************/

#include "KAR_GEO_interface.h"
#include "KAR_GEN_interface.h"

/****************************************************************************************************************
 * 																												*
 * Public Macros																								*
 * 																												*
 *****************************************************************************************************************/

#define KAR_GEO_IG_MAXPOS_X						(4175u)
#define KAR_GEO_IG_MAXPOS_Y						(4175u)
#define KAR_GEO_IG_MAXPOS_Z						(49231u)

/****************************************************************************************************************
 * 																												*
 * Public Types																									*
 * 																												*
 *****************************************************************************************************************/

typedef enum
{
	e_ErrorCode_OK = 0u,
	e_ErrorCode_OverIndexing = 1u
}e_ErrorCode_t;

/****************************************************************************************************************
 * 																												*
 * Public function declarations																					*
 * 																												*
 *****************************************************************************************************************/

/*
 *===================================================================*
 * Function name: e_PositionLookup_f
 *-------------------------------------------------------------------
 * Description:
 * The correct way for reading from the lookup table is to use this function. It's purpose is to make sure that
 * no over-indexing is done.
 *
 * ...
 *-------------------------------------------------------------------
 */
e_ErrorCode_t e_PositionLookup_f ();

/*
 *===================================================================*
 * Function name: e_Cylinder_to_Descartes_f
 *-------------------------------------------------------------------
 * Description:
 * ...
 *-------------------------------------------------------------------
 */
e_ErrorCode_t e_Cylinder_to_Descartes_f (s_GEO_ToolPosition_Cylinder cylinder_position, s_GEO_ToolPosition_Descartes *descartes_position);

#endif		// KAR_GEO_INVERSE_GEOMETRY_HANDLER_H

/* End of file KAR_GEO_InverseGeometry_handler.h */
