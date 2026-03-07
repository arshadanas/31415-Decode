package org.firstinspires.ftc.teamcode.subsystem;

import static com.acmerobotics.roadrunner.Math.lerp;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.control.Ranges.clip;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.KalmanFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.KalmanGains;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.Profiler;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Shooter {

    public static PIDGains pidGains = new PIDGains(0.0005, 0, 0, 1);
    public static KalmanGains
            rpmFilterGains = new KalmanGains(1, 2),
            derivFilterGains = new KalmanGains(3, 0),
            outputFilterGains = new KalmanGains(.02, 5);

    public static double

            MAX_VOLTAGE = 13,

            RPM_ARMING = 2700,
            RPM_IDLE = 0,

            LAUNCH_RAD_SHALLOWEST = 0.5567832093586575, // 31.901328 deg
            LAUNCH_RAD_STEEPEST = 1.0776000610289713, // 61.7419355 deg

            ANGLE_HOOD_SHALLOWEST = 360,
            ANGLE_HOOD_STEEPEST = 10,

            TOLERANCE_RPM_FILTERING = -1,
            TOLERANCE_RPM_FEEDING = 80, // TODO increase for faster feeding

            CACHE_THRESHOLD_HOOD = 0.05,
            CACHE_THRESHOLD_MOTORS = 0.001,

            DROP_RECOVERY_SCALAR = 0,
            DROP_RECOVERY_WAIT = 0.1,
            DROP_RECOVERY_TIME = 0.1;

    private final ElapsedTime rpmDropTimer = new ElapsedTime();

    private final CachedSimpleServo hood;
    private final CachedMotorEx[] motors;
    final VoltageSensor batteryVoltageSensor;

    private final KalmanFilter
            rpmFilter = new KalmanFilter(rpmFilterGains),
            derivFilter = new KalmanFilter(derivFilterGains),
            outputFilter = new KalmanFilter(outputFilterGains);
    private final PIDController controller = new PIDController(derivFilter);
    private final State setpoint = new State(), measurement = new State();

    private double currentRPM, targetRPM = RPM_ARMING, rawRPM, output;

    public void setRPM(double rpm) {
        this.targetRPM = rpm;
    }
    double getCurrentRPM() {
        return currentRPM;
    }
    Runnable boostRPM() {
        return () -> {
            boosting = true;
            rpmDropTimer.reset();
        };
    }
    private boolean boosting = false;

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
        derivFilter.setGains(derivFilterGains);
        outputFilter.setGains(outputFilterGains);
        controller.setGains(pidGains);
        Profiler.end("shooter_set_gains");

        Profiler.start("shooter_get_encoder_vel");
        rawRPM = motors[0].encoder.getCorrectedVelocity() * 60 / 28.0 * 1.35;
        Profiler.end("shooter_get_encoder_vel");

        Profiler.start("rpm kalman");
        currentRPM = rpmFilter.calculate(rawRPM);
        Profiler.end("rpm kalman");

        Profiler.start("get battery voltage");
        double voltageScalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        Profiler.end("get battery voltage");

        Profiler.start("shooter pidf");
        double rpmSetpoint =
                !feedsPending ? RPM_IDLE :
                !inLaunchZone ? RPM_ARMING :
                                targetRPM;

        controller.setTarget(setpoint.set(rpmSetpoint));
        double pidf = controller.calculate(measurement.set(currentRPM)) // pid
                + getFeedForward(rpmSetpoint) * voltageScalar; // feedforward

        if (boosting) {
            double shootTime = rpmDropTimer.seconds();
            if (shootTime > DROP_RECOVERY_WAIT + DROP_RECOVERY_TIME)
                boosting = false;
            else if (shootTime >= DROP_RECOVERY_WAIT)
                pidf += getFeedForward(AutoAim.getRPMDrop(currentRPM)) * voltageScalar * DROP_RECOVERY_SCALAR;
        }

        output =
                manualPower != 0 ?  manualPower :
                rpmSetpoint == 0 ? 0 :
                clip(inTolerance(TOLERANCE_RPM_FILTERING) ? outputFilter.calculate(pidf) : pidf, 0, 1);

        Profiler.end("shooter pidf");

        Profiler.start("shooter_motors");
        for (CachedMotorEx motor : motors) {
            motor.threshold = CACHE_THRESHOLD_MOTORS;
            motor.set(output);
        }
        Profiler.end("shooter_motors");
    }

    /**
     * <a href="https://www.desmos.com/calculator/b3jnyqoqyv">Desmos curve fit</a>
     */
    private static double getFeedForward(double rpmSetpoint) {
        return rpmSetpoint / 8367.27273 + 0.04718383310065716;
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
        telemetry.addData("Target vel (rpm)", targetRPM);
        telemetry.addData("Raw vel (rpm)", rawRPM);
        telemetry.addData("Output power [0,1]", output);
        telemetry.addLine();
        telemetry.addData("Filtered error derivative (rpm/s)", controller.getFilteredErrorDerivative());
        telemetry.addData("Raw error derivative (rpm/s)", controller.getRawErrorDerivative());
    }
}
