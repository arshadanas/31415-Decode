package org.firstinspires.ftc.teamcode.subsystem;

import static com.acmerobotics.roadrunner.Math.lerp;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.control.Ranges.clip;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.CoolerKalmanFilter;
import org.firstinspires.ftc.teamcode.control.filter.KalmanFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.KalmanGains;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.Profiler;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Shooter {

    public static PIDGains pidGains = new PIDGains(0, 0, 0, 1);
    public static KalmanGains
            rpmFilterGains = new KalmanGains(2.9, 0.03),
            kDFilterGains = new KalmanGains(),
            outputFilterGains = new KalmanGains(.02, 5);

    public static double

            POWER_A = 0.5,
            RPM_A = 3550,
            POWER_B = 1,
            RPM_B = 7875,
            MAX_VOLTAGE = 13,

            RPM_ARMING = 2700,
            RPM_IDLE = 0,

            LAUNCH_RAD_SHALLOWEST = 0.5567832093586575, // 31.901328 deg
            LAUNCH_RAD_STEEPEST = 1.0776000610289713, // 61.7419355 deg

            ANGLE_HOOD_SHALLOWEST = 360,
            ANGLE_HOOD_STEEPEST = 10,

            TOLERANCE_RPM_FILTERING = 0.001,
            TOLERANCE_RPM_FEEDING = 100, // TODO increase for faster feeding

            CACHE_THRESHOLD_HOOD = 0.05,
            CACHE_THRESHOLD_MOTORS = 0.001;

    private final CachedSimpleServo hood;
    private final CachedMotorEx[] motors;
    private final VoltageSensor batteryVoltageSensor;

    private final CoolerKalmanFilter rpmFilter = new CoolerKalmanFilter();
    private final KalmanFilter
//            rpmFilter = new KalmanFilter(rpmFilterGains),
            derivFilter = new KalmanFilter(kDFilterGains),
            outputFilter = new KalmanFilter(outputFilterGains);
    private final PIDController controller = new PIDController(derivFilter);

    private double currentRPM, currentRPMPerSec, targetRPM = RPM_ARMING, rawRPM, output;

    public void setRPM(double rpm) {
        this.targetRPM = rpm;
    }
    double getCurrentRPM() {
        return currentRPM;
    }

    /**
     * @param radians Launch angle (in radians, where 0 is horizontal, parallel to the floor)
     *                in the range [{@link #LAUNCH_RAD_SHALLOWEST}, {@link #LAUNCH_RAD_STEEPEST}]
     */
    public void setLaunchAngle(double radians) {
        hood.threshold = CACHE_THRESHOLD_HOOD; // TODO adjust when hood angle becomes continuous and differentiable
        hood.turnToAngle(lerp(
                clip(radians, LAUNCH_RAD_SHALLOWEST, LAUNCH_RAD_STEEPEST),
                LAUNCH_RAD_SHALLOWEST, LAUNCH_RAD_STEEPEST, // TODO Tune empirically
                ANGLE_HOOD_SHALLOWEST, ANGLE_HOOD_STEEPEST
        ));
    }

    private double manualPower;
    public void setManual(double power) {
        manualPower = power;
    }

    Shooter(HardwareMap hardwareMap) {
        hood = new CachedSimpleServo(hardwareMap, "hood", 0, 360).reversed();

        motors = new CachedMotorEx[]{
                new CachedMotorEx(hardwareMap, "shooter R", Motor.GoBILDA.BARE),
                new CachedMotorEx(hardwareMap, "shooter L", Motor.GoBILDA.BARE)
        };
        motors[1].setInverted(true);
        for (CachedMotorEx motor : motors)
            motor.setZeroPowerBehavior(FLOAT);

        motors[0].encoder = new CachedMotorEx(hardwareMap, "BR", Motor.GoBILDA.BARE).encoder;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    void run(boolean inLaunchZone, boolean feedsPending) {
        Profiler.start("shooter_set_gains");
        rpmFilter.setGains(rpmFilterGains);
        derivFilter.setGains(kDFilterGains);
        outputFilter.setGains(outputFilterGains);
        controller.setGains(pidGains);
        Profiler.end("shooter_set_gains");

        Profiler.start("shooter_get_encoder_vel");
        rawRPM = motors[0].encoder.getCorrectedVelocity() * 60 / 28.0 * 1.35;
        Profiler.end("shooter_get_encoder_vel");

        Profiler.start("shooter_kalman_predict");
        try {
            double[] stateEstimate = rpmFilter.predictAndCalculate(rawRPM, 0);
            currentRPM = stateEstimate[0];
            currentRPMPerSec = stateEstimate[1];
        } catch (Exception ignored) {}
        Profiler.end("shooter_kalman_predict");

        Profiler.start("shooter_set_vars");
        double rpmSetpoint =
                !feedsPending ? RPM_IDLE : // change to EMPTY.numOccurrencesIn(handler.container.artifacts) == 3 ?
                !inLaunchZone ? RPM_ARMING :
                                targetRPM;

        double voltageScalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        Profiler.end("shooter_set_vars");

        Profiler.start("shooter_set_controller");
        controller.setTarget(new State(rpmSetpoint));
        Profiler.end("shooter_set_controller");


        Profiler.start("shooter_pid_new");
        double pid = controller.calculate(new State(currentRPM, currentRPMPerSec));
        double feedforward = lerp(rpmSetpoint, RPM_A, RPM_B, POWER_A, POWER_B) * voltageScalar;
        double pidf = pid + feedforward;

        output = manualPower != 0 ? manualPower : clip(
                        inTolerance(TOLERANCE_RPM_FILTERING) ?
                                    outputFilter.calculate(pidf) :
                                    pidf
                , 0, 1);

        Profiler.end("shooter_pid_new");

        Profiler.start("shooter_motors");
        for (CachedMotorEx motor : motors) {
            motor.threshold = CACHE_THRESHOLD_MOTORS;
            motor.set(output);
        }
        Profiler.end("shooter_motors");
    }

    boolean inTolerance(double rpmTolerance) {
        return abs(targetRPM - currentRPM) <= rpmTolerance;
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("SHOOTER",
                inTolerance(TOLERANCE_RPM_FILTERING) ?  "RPM in filtering tolerance" :
                inTolerance(TOLERANCE_RPM_FEEDING) ?    "RPM in feeding tolerance" :
                                                        "RPM out of tolerance"
        );
        telemetry.addLine();
        telemetry.addData("Current vel (rpm)", currentRPM);
        telemetry.addData("Current accel (rpm/s)", currentRPMPerSec);
        telemetry.addData("Target vel (rpm)", targetRPM);
        telemetry.addData("Raw vel (rpm)", rawRPM);
        telemetry.addData("Output power [0,1]", output);
        telemetry.addLine();
        telemetry.addData("Filtered error derivative (rpm/s)", controller.getFilteredErrorDerivative());
        telemetry.addData("Raw error derivative (rpm/s)", controller.getRawErrorDerivative());
    }
}
