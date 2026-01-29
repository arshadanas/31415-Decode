package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.WraparoundKalmanFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.KalmanGains;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;

import java.util.ArrayList;

public final class Turret {

    public static KalmanGains filterGains = new KalmanGains();
    public static PIDGains pidGains = new PIDGains();

    public static double
            TURRET_ABSOLUTE_OFFSET = 1.8659156366775742,
            QUADRATURE_RAD_PER_TICK = 2 * PI / (4 * 145.090909091),
            WRAPAROUND_POSITION = toRadians(180),
            DEFAULT_TOLERANCE = toRadians(3),
            HOMING_TOLERANCE = DEFAULT_TOLERANCE;

    private final CachedMotorEx motor;
    private final AnalogSensor absoluteEnc;

    /**
     * In radians
     */
    private double position, quadratureOffset, target, absolutePosition;
    public void setTarget(double target) {
        this.target = normalizeRadians(target);
    }

    private final WraparoundKalmanFilter derivFilter = new WraparoundKalmanFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    Turret(HardwareMap hardwareMap) {
        motor = new CachedMotorEx(hardwareMap,  "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(true);

        motor.encoder = new CachedMotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_1150).encoder;
        motor.encoder.setDirection(REVERSE);
        motor.encoder.setDistancePerPulse(QUADRATURE_RAD_PER_TICK);
        motor.encoder.reset();

        absoluteEnc = new AnalogSensor(hardwareMap, "elc", 2 * PI);
    }

    void run(double intakePower, ArrayList<Integer> feedingOrder, boolean inShootingZone) {

        if (intakePower == 0 && feedingOrder.isEmpty() && atPosition(target, HOMING_TOLERANCE))
            recalibrateQuadrature();

        position = motor.encoder.getDistance() + quadratureOffset;

        derivFilter.setGains(filterGains);
        controller.setGains(pidGains);

        controller.setTarget(new State(normalizeRadians(target + PI - WRAPAROUND_POSITION)));
        motor.set(controller.calculate(new State(position + PI - WRAPAROUND_POSITION)));
    }

    private boolean atPosition(double target, double tolerance) {
        return abs(getError(target)) <= tolerance;
    }

    private double getError(double target) {
        return normalizeRadians(target - position);
    }

    private void recalibrateQuadrature() {
        absolutePosition = normalizeRadians(-absoluteEnc.getReading() + Turret.TURRET_ABSOLUTE_OFFSET);
        quadratureOffset = normalizeRadians(absolutePosition - position);
    }

    void printTo(Telemetry telemetry) {
        telemetry.addLine("TURRET");
        telemetry.addLine();
        telemetry.addData("Position (deg)", toDegrees(position));
        telemetry.addData("Target (deg)", toDegrees(target));
        telemetry.addLine();
        telemetry.addData("Error (deg)", toDegrees(target - position));
        telemetry.addData("Filtered error derivative (deg/s)", toDegrees(controller.getFilteredErrorDerivative()));
        telemetry.addData("Raw error derivative (deg/s)", toDegrees(controller.getRawErrorDerivative()));
        telemetry.addLine();
        telemetry.addData("Absolute position (deg)", toDegrees(absolutePosition));
    }
}
