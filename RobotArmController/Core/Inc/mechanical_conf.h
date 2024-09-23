#ifndef MECHANICAL_CONF_H_
#define MECHANICAL_CONF_H_

// 1.8° = 0.03141593rad
#define MOTOR_STEP_ANGLE_R (0.03141593)
// 3.5arcmin = 0.05833333° = 0.00101811rad
#define MOTOR_STEP_ANGLE_PHI (0.00101811)
// 1.8° = 0.03141593rad
#define MOTOR_STEP_ANGLE_Z (0.03141593)

// 3.95cm = 0.0395m (/rad)
#define JOINT_TRANSFORM_FACTOR_R (0.0395)
// 1rad (/rad)
#define JOINT_TRANSFORM_FACTOR_PHI (1.0)
// 2mm / (2 * pi) = 0.00031831m (/rad)
#define JOINT_TRANSFORM_FACTOR_Z (0.00031831)

#define MOTOR_TOTAL_STEP_R 1040
#define MOTOR_TOTAL_STEP_PHI 13800
#define MOTOR_TOTAL_STEP_Z 11500

// [rad]
#define MOTOR_RANGE_R (MOTOR_TOTAL_STEP_R * MOTOR_STEP_ANGLE_R)
#define MOTOR_RANGE_PHI (MOTOR_TOTAL_STEP_PHI * MOTOR_STEP_ANGLE_PHI)
#define MOTOR_RANGE_Z (MOTOR_TOTAL_STEP_Z * MOTOR_STEP_ANGLE_Z)

// SI
#define JOINT_RANGE_R (MOTOR_RANGE_R * JOINT_TRANSFORM_FACTOR_R)
#define JOINT_RANGE_PHI (MOTOR_RANGE_PHI * JOINT_TRANSFORM_FACTOR_PHI)
#define JOINT_RANGE_Z (MOTOR_RANGE_Z * JOINT_TRANSFORM_FACTOR_Z)

#endif /* MECHANICAL_CONF_H_ */
