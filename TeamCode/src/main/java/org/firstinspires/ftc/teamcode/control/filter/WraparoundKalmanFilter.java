package org.firstinspires.ftc.teamcode.control.filter;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import org.firstinspires.ftc.teamcode.control.gainmatrix.KalmanGains;

public final class WraparoundKalmanFilter implements Filter {

    private KalmanGains gains;

    public WraparoundKalmanFilter() {
        this(new KalmanGains());
    }

    public WraparoundKalmanFilter(KalmanGains gains) {
        setGains(gains);
    }

    public void setGains(KalmanGains gains) {
        this.gains = gains;
    }

    private double x = 0; // your initial state
    private double p = 1; // your initial covariance guess
    private double K = 1; // your initial Kalman gain guess
    private double x_previous = x;
    private double p_previous = p;
    private double u = 0;
    private double z = 0;

    public void reset() {
        x = 0;
        p = 1;
        K = 1;
        x_previous = x;
        p_previous = p;
        u = 0;
        z = 0;
    }

    @Override
    public double calculate(double newValue) {

        newValue = x_previous + normalizeRadians(newValue - x_previous);

        u = 0;
        // Predict our new state
        x = x_previous + u;

        // Predict our new covariance
        p = p_previous + gains.Q;

        // Update our kalman gain
        K = p/(p + gains.R);

        // Get a new reading from sensor
        z = newValue; // Pose Estimate from April Tag / Distance Sensor

        // Update the state
        x = x + K * (z - x);

        // Update the covariance
        p = (1 - K) * p;

        x_previous = x;
        p_previous = p;

        return x;

    }
}
