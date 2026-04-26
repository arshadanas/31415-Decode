package org.firstinspires.ftc.teamcode.control.motion;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;

@Config
public final class PIDDriver {

    public static PIDGains
            xyGains = new PIDGains(
            0.0175,
                    0,
                    0,
                    1
            ),
            rotGains = new PIDGains(
                    1.25,
                    0.75,
                    0,
                    1
            );

    public static double STRAFE_MULTIPLIER = 1;

    public static EditablePose admissibleError = new EditablePose(0.01, 0.01, 0.001);

    private final PIDController
            xController = new PIDController(),
            yController = new PIDController(),
            rotController = new PIDController();

    private final State
            xSetpoint = new State(),
            ySetpoint = new State(),
            rotSetpoint = new State(),
            xMeasurement = new State(),
            yMeasurement = new State(),
            rotMeasurement = new State();

    public void reset() {
        xController.reset();
        yController.reset();
        rotController.reset();
    }

    public DriverOutput driveTo(EditablePose current, EditablePose target) {

        double xError = target.x - current.x;
        double yError = target.y - current.y;
        double headingError = normalizeRadians(target.heading - current.heading);

        xController.setGains(xyGains);
        yController.setGains(xyGains);
        rotController.setGains(rotGains);

        xController.setTarget(xSetpoint.set(target.x));
        yController.setTarget(ySetpoint.set(target.y *1));
        rotController.setTarget(rotSetpoint.set(headingError + current.heading));

        double x = xController.calculate(xMeasurement.set(current.x));
        double y = yController.calculate(yMeasurement.set(current.y *1));
        double rot = rotController.calculate(rotMeasurement.set(current.heading));

        return new DriverOutput(
        new EditablePose(x, y * STRAFE_MULTIPLIER, -rot),
        abs(xError) <= admissibleError.x &&
                abs(yError) <= admissibleError.y &&
                abs(headingError) <= admissibleError.heading
        );
    }

    public static class DriverOutput {

        public DriverOutput(EditablePose drivePower, boolean withinError) {
            this.drivePower = drivePower;
            this.withinError = withinError;
        }

        public final boolean withinError;
        public final EditablePose drivePower;
    }
}
