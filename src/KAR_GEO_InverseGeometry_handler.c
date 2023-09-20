/**
 * @file KAR_GEO_InverseGeometry_handler.c
 *
 * "Makes coordinate-system changing calculations"
 *
 * $Author: Erdei Sándor
 * $Date: 2023-08-09
 * $Revision: $
 *
 * @copyright LEGO kör - INDACT project
 *
 * TODO
 *
 */

/*# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
 #																												#
 #		* Detailed description *																				#
 #																												#
 # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # */

/****************************************************************************************************************
 * 																												*
 * Includes																										*
 * 																												*
 *****************************************************************************************************************/

#include "KAR_GEO_InverseGeometry_handler.h"

/****************************************************************************************************************
 * 																												*
 * Local Macros																								*
 * 																												*
 *****************************************************************************************************************/

#define L_LOOKUP_SIZE							(434u)

#define L_XPOS_FISRT_NEGATIVE_NUMBERS_INDEX		(131u)
#define L_XPOS_LAST_NEGATIVE_NUMBERS_INDEX		(389u)
#define L_YPOS_FISRT_NEGATIVE_NUMBERS_INDEX		(260u)

/****************************************************************************************************************
 * 																												*
 * Local Constants																								*
 * 																												*
 *****************************************************************************************************************/

/*
 * Const values will be stored in flash, instead of RAM
 * Static is used here to limit the scope of the lookup table.
 * Scope outside the file can let the linker to make decisions about the table placement.
 */
static const uint8_t lookup_table[L_LOOKUP_SIZE] = {

};

/****************************************************************************************************************
 * 																												*
 * Public Types																									*
 * 																												*
 *****************************************************************************************************************/



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
 * INPUT:
 * @param (s_GEO_ToolPosition_Cylinder) position: The position in cylinder coordinate system
 * @param (s_GEO_ToolPosition_Descartes*) return_position_ptr: @param position in Descartes coordinate system.
 *
 * OUTPUT:
 * @retval (e_ErrorCode_t) e_errorCode: Indicating successful read from lookup table.
 *-------------------------------------------------------------------
 */
e_ErrorCode_t e_PositionLookup_f ()
{
	e_ErrorCode_t errorCode = e_ErrorCode_OK;

	//...

	return errorCode;
}

/*
 *===================================================================*
 * Function name: e_Cylinder_to_Descartes_f
 *-------------------------------------------------------------------
 * Description:
 * The correct way for reading from the lookup table is to use this function. It's purpose is to make sure that
 * no over-indexing is done.
 *
 * INPUT:
 * @param (s_GEO_ToolPosition_Cylinder) cylinder_position: Position which needs to be converted.
 *
 * OUTPUT:
 * @retval (e_ErrorCode_t) errorCode: Indicating successful read from lookup table.
 *-------------------------------------------------------------------
 */
e_ErrorCode_t e_Cylinder_to_Descartes_f (s_GEO_ToolPosition_Cylinder cylinder_position, s_GEO_ToolPosition_Descartes *descartes_position)
{
	e_ErrorCode_t e_errorCode = e_ErrorCode_OK;

	//...

	return e_errorCode;
}


/* End of file KAR_GEO_InverseGeometry_handler.c */
