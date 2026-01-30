package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

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

import java.util.ArrayList;

public final class Shooter {

    public static PIDGains pidGains = new PIDGains();
    public static KalmanGains
            rpmFilterGains = new KalmanGains(),
            pidFilterGains = new KalmanGains(),
            outputFilterGains = new KalmanGains();

    public static double
            RPM_MAX = 7830, // TODO MEASURE
            RPM_ARMING = 2700,
            RPM_IDLE = 1620,

            LAUNCH_RAD_SHALLOWEST = toRadians(31.901328),
            LAUNCH_RAD_STEEPEST = toRadians(61.7419355),

            ANGLE_HOOD_SHALLOWEST = 360,
            ANGLE_HOOD_STEEPEST = 10,

            TOLERANCE_RPM_SHOOTING = 10,
            TOLERANCE_RPM_FEEDING = 10; // TODO increase for faster feeding

    private final CachedSimpleServo hood;
    private final CachedMotorEx[] motors;

    private final KalmanFilter
            rpmFilter = new KalmanFilter(rpmFilterGains),
            derivFilter = new KalmanFilter(pidFilterGains),
            outputFilter = new KalmanFilter(outputFilterGains);
    private final PIDController controller = new PIDController(derivFilter);

    private double currentRPM, targetRPM, rawRPM;

    public void setRPM(double rpm) {
        this.targetRPM = rpm;
    }
    public void setRadPerSec(double radPerSec) {
        setRPM(radPerSec * 2 * PI / 60);
    }
    public void setLaunchVel(double inPerSec) {
        setRPM(29.68064 * inPerSec - 0.445157); // TODO Tune empirically
    }

    public void setLaunchAngle(double radians) {
        hood.turnToAngle(Math.lerp(
                radians,
                LAUNCH_RAD_SHALLOWEST, LAUNCH_RAD_STEEPEST,
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

    void run(boolean inShootingZone, ArrayList<Integer> feedingOrder) {
        rpmFilter.setGains(rpmFilterGains);
        derivFilter.setGains(pidFilterGains);
        controller.setGains(pidGains);

        rawRPM = motors[0].encoder.getCorrectedVelocity() * 60 / 28.0 * 1.35;
        currentRPM = rpmFilter.calculate(rawRPM);

        double rpmSetpoint =
                inShootingZone ?            targetRPM :
                !feedingOrder.isEmpty() ?   RPM_ARMING : // change to EMPTY.numOccurrencesIn(handler.container.artifacts) < 3 ?
                                            RPM_IDLE;

        controller.setTarget(new State(rpmSetpoint));
        double pid = controller.calculate(new State(currentRPM));
        double feedforward = rpmSetpoint / RPM_MAX;
        double autoPower = pid + feedforward;

        double power = manualPower != 0 ? manualPower : clip(
                        inTolerance(TOLERANCE_RPM_SHOOTING) ?
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
                inTolerance(TOLERANCE_RPM_SHOOTING) ?
                        "RPM in shooting tolerance" :
                        "RPM out of shooting tolerance"
        );
        telemetry.addLine();
        telemetry.addData("Velocity (rpm)", currentRPM);
        telemetry.addData("Target vel (rpm)", targetRPM);
        telemetry.addData("Raw velocity (rpm)", rawRPM);
        telemetry.addLine();
        telemetry.addData("Filtered error derivative (rpm/s)", controller.getFilteredErrorDerivative());
        telemetry.addData("Raw error derivative (rpm/s)", controller.getRawErrorDerivative());
    }
}
