#ifndef __IMU_HEADER_H__
#define __IMU_HEADER_H__

#include "common.h"

#ifdef CORE_CM4

void imu_init(void);
void imu_start(void);
void imu_update_packet_orientation(void);
void imu_quat_to_roll_pitch_yaw(double qw, double qx, double qy, double qz, double *x_roll, double *y_pitch, double *z_yaw);

#endif  // #ifdef CORE_CM4

#endif  // #ifndef __IMU_HEADER_H__
