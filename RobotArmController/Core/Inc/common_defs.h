/**
 * @brief This file contains useful type definitions.
 *
 * @author Péter Varga (petervarga0018@gmail.com)
 * @date 2024-12-26
 */

#ifndef COMMON_DEFS_H_
#define COMMON_DEFS_H_

#include <stdint.h>

typedef int32_t MC_Step_t;

typedef struct PositionCylindrical
{
    MC_Step_t r;
    MC_Step_t phi;
    MC_Step_t z;
} PositionCylindrical_t;

#endif /* COMMON_DEFS_H_ */
