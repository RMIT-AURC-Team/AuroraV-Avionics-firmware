#ifndef _KALMANFILTER_H
#define _KALMANFILTER_H

#include "arm_math.h"

typedef struct KalmanFilter {
  arm_matrix_instance_f32 A; // State transition matrix
  arm_matrix_instance_f32 H; // Measurement matrix
  arm_matrix_instance_f32 Q; // Process noise covariance
  arm_matrix_instance_f32 R; // Measurement noise covariance
  arm_matrix_instance_f32 P; // Estimate error covariance
  arm_matrix_instance_f32 K; // Kalman gain
  arm_matrix_instance_f32 x; // State estimate
  arm_matrix_instance_f32 z; // Measurement
  void (*update)(struct KalmanFilter*, arm_matrix_instance_f32*);
} KalmanFilter;

void KalmanFilter_init(KalmanFilter*);
void KalmanFilter_update(KalmanFilter*, arm_matrix_instance_f32*);

#endif
