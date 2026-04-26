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

    public static KalmanGains filterGains = new KalmanGains(.5, 10);
    public static PIDGains pidGains = new PIDGains(1, 1.5, 0.05, 0.2);

    public static double
            CACHE_THRESHOLD_MOTOR = 0.05,
            TURRET_ABSOLUTE_OFFSET = 0.12375971059596157,
            QUADRATURE_RAD_PER_TICK = 2 * PI / (4 * 145.090909091),
            TOLERANCE_NO_RECALIBRATING = toRadians(25),
            TOLERANCE_FEEDING = toRadians(5); // TODO can be increased for faster feeds

    final CachedMotorEx motor;
    private final AnalogSensor absoluteEnc;

    /**
     * In radians
     */
    private double relativeEncoderOffset, absolutePosition, output;
    private final State current = new State(), target = new State();

    /**
     * 0 = turret facing forward
     */
    public void setTarget(double radians) {
        this.target.x = normalizeRadians(radians + PI);
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
        motor.threshold = CACHE_THRESHOLD_MOTOR;

        current.x = motor.encoder.getDistance() + relativeEncoderOffset;
        absolutePosition = normalizeRadians(-absoluteEnc.getReading() + Turret.TURRET_ABSOLUTE_OFFSET);

        // Not feeding/shooting, robot is idle
        if (!feedsPending) {
            motor.set(0); // dont move turret
            controller.reset(); // reset integrator + derivFilter

            // only recalibrate relative encoder if turret is FAR from the wraparound position (PI/-PI)
            if (abs(normalizeRadians(PI - absolutePosition)) > TOLERANCE_NO_RECALIBRATING)
                relativeEncoderOffset += normalizeRadians(absolutePosition - current.x);
            
            return;
        }


        derivFilter.setGains(filterGains);
        controller.setGains(pidGains);

        controller.setTarget(target);
        motor.set(output = controller.calculate(current));

    }

    boolean inTolerance() {
        return abs(normalizeRadians(target.x - current.x)) <= TOLERANCE_FEEDING;
    }

    void printTo(Telemetry telemetry) {
        telemetry.addLine("TURRET");
        telemetry.addLine();
        telemetry.addData("Position (deg)", toDegrees(current.x));
        telemetry.addData("Target (deg)", toDegrees(target.x));
        telemetry.addLine();
        telemetry.addData("Error (deg)", toDegrees(target.x - current.x));
        telemetry.addData("Filtered error derivative (deg/s)", toDegrees(controller.getFilteredErrorDerivative()));
        telemetry.addData("Raw error derivative (deg/s)", toDegrees(controller.getRawErrorDerivative()));
        telemetry.addLine();
        telemetry.addData("Absolute position (deg)", toDegrees(absolutePosition));
        telemetry.addData("Absolute position (rad)", (absolutePosition));
        telemetry.addLine();
        telemetry.addData("Turret motor output power", output);

    }
}
