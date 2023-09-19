#ifndef EKF_H_
#define EKF_H_

#include <math.h>

// Quaternion typedef
typedef struct Quaternion_TypeDef
{
    double w, x, y, z;

} Quaternion_TypeDef;

// Attitude typedef
typedef struct Attitude_TypeDef
{
    double roll,
	pitch,
	yaw;
    Quaternion_TypeDef q;
} Attitude_TypeDef;

// State typedef
typedef struct State_TypeDef
{
    Quaternion_TypeDef q;

    double bias_g_x,
	bias_g_y,
	bias_g_z;
} State_TypeDef;

// Gyro Data typedef
typedef struct Gyro_TypeDef
{
    double g_x,
	g_y,
	g_z;
} Gyro_TypeDef;

/**
 * @brief Convert euler angles to quaternion
 */
void to_quaternion(Attitude_TypeDef *att);

/**
 * @brief Convert a quaternion to euler angles
 */
void to_euler(Attitude_TypeDef *att);

/**
 * @brief Kalman filter predict step
 */
State_TypeDef predict(State_TypeDef *x, Gyro_TypeDef *w, double ts);


/**
 * @brief Kalman filter update step
 */

#endif
