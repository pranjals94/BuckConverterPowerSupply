
#include "digitalFilter.h"
#include <math.h>

void kalman_init(KalmanFilter* kf, float estimate, float err_estimate, float err_measurement, float alpha_lowpass) {
	kf->estimate = estimate;
	kf->error_estimate = err_estimate;
	kf->error_measure = err_measurement;
	kf->kalman_gain = 0.0;
	kf->alpha_lowpass = alpha_lowpass;
	kf->lowpass_output = 0.0;
}

float kalman_update(KalmanFilter* kf, float measurement) {
	// Kalman gain calculation
	kf->kalman_gain = kf->error_estimate / (kf->error_estimate + kf->error_measure);
	// Update estimate with measurement
	kf->estimate = kf->estimate + kf->kalman_gain * (measurement - kf->estimate);
	
	// Adaptive error estimate: Boost on big changes
	float diff = fabs(measurement - kf->estimate);

	if (diff > 10.0) {
		// Increase trust in new measurement
		kf->error_estimate += 8.0;
		if (kf->error_estimate > 15.0) kf->error_estimate = 15.0;  // clamp max
	}
	
	// Normal decay of error estimate to regain smoothing over time
	kf->error_estimate = (1.0 - kf->kalman_gain) * kf->error_estimate;
	
	// Optional: Exponential smoothing (low-pass filter)
	kf->lowpass_output = kf->alpha_lowpass * kf->estimate + (1.0 - kf->alpha_lowpass) * kf->lowpass_output;
	
	return kf->lowpass_output;
}
