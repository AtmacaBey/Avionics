float kalman_filter (float input)
{
float kalman_new = kalman_old; // old value is taken 
float cov_new = cov_old + 0.3; //the new covariance value is determined. Q = 50
float kalman_gain = cov new / (cov new + 0.9); // Kalman gain calculated. R = 0.9 
float kalman calculated = kalman new + (kalman gain * (input - kalman new)); //Kalman value is calculated
cov_new = (1 - kalman_gain) * cov_old; // The new covariance value is calculated 
cov old = cov new; // New values are saved for use in the next cycle.
kalman old = kalman calculated;

return kalman_calculated;
}
