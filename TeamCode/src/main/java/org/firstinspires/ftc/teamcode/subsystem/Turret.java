package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.FORWARD;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.KalmanFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.KalmanGains;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;

@Config
public final class Turret {

    public static KalmanGains filterGains = new KalmanGains(.5, 50);
    public static PIDGains pidGains = new PIDGains(0, 0, 0, 0.25);

    public static double
            TURRET_ABSOLUTE_OFFSET = 0,
            QUADRATURE_RAD_PER_TICK = 2 * PI / (4 * 145.090909091),
            WRAPAROUND_POSITION = toRadians(0),
            TOLERANCE_FEEDING = toRadians(3); // TODO can be increased for faster feeds

    final CachedMotorEx motor;
    private final AnalogSensor absoluteEnc;

    /**
     * In radians
     */
    private double position, quadratureOffset, target, absolutePosition, output;
    public void setTarget(double target) {
        this.target = normalizeRadians(target);
    }

    private final KalmanFilter derivFilter = new KalmanFilter(filterGains, true);
    private final PIDController controller = new PIDController(derivFilter);

    Turret(HardwareMap hardwareMap) {
        motor = new CachedMotorEx(hardwareMap,  "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(true);
        motor.setZeroPowerBehavior(BRAKE);

        motor.encoder = new CachedMotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_1150).encoder;
        motor.encoder.setDirection(FORWARD);
        motor.encoder.setDistancePerPulse(QUADRATURE_RAD_PER_TICK);
        motor.encoder.reset();

        absoluteEnc = new AnalogSensor(hardwareMap, "elc", 2 * PI);

        run(false);
    }

    void run(boolean feedsPending) {

        position = motor.encoder.getDistance() + quadratureOffset;

        if (!feedsPending) {
            motor.set(0);
            controller.reset();
            absolutePosition = normalizeRadians(-absoluteEnc.getReading() + Turret.TURRET_ABSOLUTE_OFFSET);
            quadratureOffset += normalizeRadians(absolutePosition - position);
            return;
        }

        derivFilter.setGains(filterGains);
        controller.setGains(pidGains);

        controller.setTarget(new State(normalizeRadians(target + PI - WRAPAROUND_POSITION)));
        motor.set(output = controller.calculate(new State(position + PI - WRAPAROUND_POSITION)));
    }

    boolean inTolerance(double tolerance) {
        return abs(normalizeRadians(target - position)) <= tolerance;
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

        telemetry.addData("Turret motor output power", output);

    }
}
