package org.firstinspires.ftc.teamcode.control.filter;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math4.legacy.filter.DefaultMeasurementModel;
import org.apache.commons.math4.legacy.filter.DefaultProcessModel;
import org.apache.commons.math4.legacy.filter.MeasurementModel;
import org.apache.commons.math4.legacy.filter.ProcessModel;
import org.apache.commons.math4.legacy.linear.Array2DRowRealMatrix;
import org.apache.commons.math4.legacy.linear.ArrayRealVector;
import org.apache.commons.math4.legacy.linear.RealMatrix;
import org.apache.commons.math4.legacy.linear.RealVector;
import org.apache.commons.math4.legacy.filter.KalmanFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.KalmanGains;

public class CoolerKalmanFilter {

    private KalmanGains gains = new KalmanGains(0.1, 0.1);
    public void setGains(KalmanGains gains) {
        this.gains = gains;
    }

    private final static double initialDt = 0.03;

    // A = [ 1 dt ]
    //     [ 0  1 ]
    private final RealMatrix stateTransitionA = new Array2DRowRealMatrix(new double[][] {
            { 1, initialDt },
            { 0,         1 }
    }, false);

    // B = [ dt ]
    //     [ 1  ]
    private final RealMatrix controlB = new Array2DRowRealMatrix(new double[][] {
            { Math.pow(initialDt, 2d) / 2d },
            {         initialDt }
    }, false);

    // H = [ 1 0 ]
    private final RealMatrix measurementModelH = new Array2DRowRealMatrix(new double[][] {
            { 1d, 0d }
    }, false);

    // x = [ 0 0 ]
    private final RealVector stateEstimateX = new ArrayRealVector(new double[] { 0, 0 }, false);

    // Q = [ dt^4/4 dt^3/2 ]
    //     [ dt^3/2 dt^2   ]
    private final RealMatrix processNoiseQ = new Array2DRowRealMatrix(new double[][] {
            { 0, 0 },
            { 0, 0 }
    }, false);

    private void updateProcessNoise(double dt){
        double scalar = Math.pow(gains.Q, 2);
        processNoiseQ.setEntry(0, 0, Math.pow(dt, 4d) / 4d * scalar);
        processNoiseQ.setEntry(1, 0, Math.pow(dt, 3d) / 2d * scalar);
        processNoiseQ.setEntry(0, 1, Math.pow(dt, 3d) / 2d * scalar);
        processNoiseQ.setEntry(1, 1, Math.pow(dt, 2d) * scalar);
    }

    // P0 = [ 1 1 ]
    //      [ 1 1 ]
    private final RealMatrix errorCovarianceP0 = new Array2DRowRealMatrix(new double[][] {
            { 1, 1 },
            { 1, 1 }
    }, false);

    // R = [ measurementNoise^2 ]
    private final RealMatrix measurementNoiseR = new Array2DRowRealMatrix(new double[] { Math.pow(gains.R, 2) });

    // Our control input
    private final RealVector controlInputU = new ArrayRealVector(new double[] { 0.1d }, false);

    private final ProcessModel pm = new DefaultProcessModel(stateTransitionA, controlB, processNoiseQ, stateEstimateX, errorCovarianceP0);
    private final MeasurementModel mm = new DefaultMeasurementModel(measurementModelH, measurementNoiseR);
    private final KalmanFilter filter = new KalmanFilter(pm, mm);

    public CoolerKalmanFilter() {
        updateProcessNoise(initialDt);
    }

    private final RealVector realMeasurement = new ArrayRealVector(new double[]{ 0.0d });

    private final ElapsedTime timer = new ElapsedTime();

    public double[] predictOnly(double controlInput){
        predict(controlInput);
        return filter.getStateEstimation();
    }

    public double[] predictAndCalculate(double measurement, double controlInput) {
        predict(controlInput);

        realMeasurement.setEntry(0, measurement);
        filter.correct(realMeasurement);

        return filter.getStateEstimation();
    }

    private void predict(double controlInput) {
        double dt = Math.max(timer.seconds(), 0.000000001);
        timer.reset();

        stateTransitionA.setEntry(0, 1, dt);
        controlB.setEntry(0, 0, Math.pow(dt, 2d) / 2d );
        controlB.setEntry(1, 0, dt);
        updateProcessNoise(dt);
        measurementNoiseR.setEntry(0,0, Math.pow(gains.R, 2));

        controlInputU.setEntry(0, controlInput);
        filter.predict(controlInputU);
    }

//    public static void main(String... args) throws InterruptedException {
//        CoolerKalmanFilter filter = new CoolerKalmanFilter();
//        Thread.sleep(2000);
//        filter.calculate();
//        System.out.println(filter.A);
//    }


}
