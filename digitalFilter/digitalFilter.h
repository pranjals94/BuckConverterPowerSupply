#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
	float estimate;
	float error_estimate;
	float error_measure;
	float kalman_gain;
	float alpha_lowpass;
	float lowpass_output;
} KalmanFilter;

void kalman_init(KalmanFilter* kf, float estimate, float err_estimate, float err_measurement, float alpha_lowpass);
float kalman_update(KalmanFilter* kf, float measurement);

#endif


