package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.control.Ranges.clip;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Math;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.KalmanFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.KalmanGains;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Shooter {

    /**
     * Inverse of {@link #getLaunchVel}
     * @return Angular velocity of shooter wheel at launch, in rpm
     */
    public static double getLaunchRPM(double inPerSec) {
        return 29.68064 * inPerSec - 0.445157; // TODO Tune empirically
    }

    /**
     * Inverse of {@link #getLaunchRPM}
     * @return Linear velocity of {@link Artifact} at launch, in inches/sec
     */
    public static double getLaunchVel(double rpm) {
        return (rpm + 0.445157) / 29.68064; // TODO Tune empirically
    }

    public static PIDGains pidGains = new PIDGains();
    public static KalmanGains
            rpmFilterGains = new KalmanGains(),
            pidFilterGains = new KalmanGains(),
            outputFilterGains = new KalmanGains();

    public static double
            RPM_MAX = 7521.4285714285725,
            RPM_ARMING = 2700,
            RPM_IDLE = 1620,

            LAUNCH_RAD_SHALLOWEST = toRadians(31.901328),
            LAUNCH_RAD_STEEPEST = toRadians(61.7419355),

            ANGLE_HOOD_SHALLOWEST = 360,
            ANGLE_HOOD_STEEPEST = 10,

            TOLERANCE_RPM_FILTERING = 10,
            TOLERANCE_RPM_FEEDING = 10, // TODO increase for faster feeding

            RPM_NEAR = 4000,
            RPM_FAR = 6000,
            LAUNCH_RAD_NEAR = LAUNCH_RAD_STEEPEST,
            LAUNCH_RAD_FAR = LAUNCH_RAD_SHALLOWEST;

    private final CachedSimpleServo hood;
    private final CachedMotorEx[] motors;

    private final KalmanFilter
            rpmFilter = new KalmanFilter(rpmFilterGains),
            derivFilter = new KalmanFilter(pidFilterGains),
            outputFilter = new KalmanFilter(outputFilterGains);
    private final PIDController controller = new PIDController(derivFilter);

    private double currentRPM, targetRPM = RPM_ARMING, rawRPM;

    public void setRPM(double rpm) {
        this.targetRPM = rpm;
    }
    public void setLaunchVel(double inPerSec) {
        setRPM(getLaunchRPM(inPerSec));
    }

    /**
     * @param radians Launch angle (in radians, where 0 is horizontal, parallel to the floor)
     *                in the range [{@link #LAUNCH_RAD_SHALLOWEST}, {@link #LAUNCH_RAD_STEEPEST}]
     */
    public void setLaunchAngle(double radians) {
        hood.turnToAngle(Math.lerp(
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
    }

    void run(boolean inLaunchZone, boolean feedsPending) {
        rpmFilter.setGains(rpmFilterGains);
        derivFilter.setGains(pidFilterGains);
        controller.setGains(pidGains);

        rawRPM = motors[0].encoder.getCorrectedVelocity() * 60 / 28.0 * 1.35;
        currentRPM = rpmFilter.calculate(rawRPM);

        double rpmSetpoint =
                !feedsPending ? RPM_IDLE : // change to EMPTY.numOccurrencesIn(handler.container.artifacts) == 3 ?
                inLaunchZone ?  targetRPM :
                                RPM_ARMING;

        controller.setTarget(new State(rpmSetpoint));
        double pid = controller.calculate(new State(currentRPM));
        double feedforward = rpmSetpoint / RPM_MAX;
        double autoPower = pid + feedforward;

        double power = manualPower != 0 ? manualPower : clip(
                        inTolerance(TOLERANCE_RPM_FILTERING) ?
                                    outputFilter.calculate(autoPower) :
                                    autoPower
                , 0, 1);
        for (CachedMotorEx motor : motors)
            motor.set(power);
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
        telemetry.addData("Current (rpm)", currentRPM);
        telemetry.addData("Target (rpm)", targetRPM);
        telemetry.addData("Raw (rpm)", rawRPM);
        telemetry.addLine();
        telemetry.addData("Filtered error derivative (rpm/s)", controller.getFilteredErrorDerivative());
        telemetry.addData("Raw error derivative (rpm/s)", controller.getRawErrorDerivative());
    }
}
