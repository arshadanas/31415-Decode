package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.control.Ranges.clip;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.max;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Config
public final class MecanumDrivetrain {

    public static double
            SLOW_FACTOR = 0.3,
            MOTOR_CACHING_THRESHOLD = 0.05;

    private final Follower drivetrain;

    MecanumDrivetrain(HardwareMap hardwareMap, Pose pose) {
        Constants.mecanumConstants.setMotorCachingThreshold(MOTOR_CACHING_THRESHOLD);
        drivetrain = Constants.createFollower(hardwareMap);
        try {
            drivetrain.getPoseTracker().resetIMU();
        } catch (InterruptedException ignored) {}

        drivetrain.setStartingPose(pose);

        update();
    }

    /**
     * Drive robot with control stick inputs
     *
     * @param strafeCommand    positive = strafing right
     * @param forwardCommand    positive = forward
     * @param turnCommand positive = clockwise
     * @param useSlowMode drives robot at {@link #SLOW_FACTOR} of full speed
     */
    public void run(double strafeCommand, double forwardCommand, double turnCommand, boolean useSlowMode, boolean isRedAlliance) {

        forwardCommand = clip(forwardCommand, -1, 1);
        strafeCommand = clip(strafeCommand, -1, 1);
        turnCommand = clip(turnCommand, -1, 1);

        if (useSlowMode) {
            forwardCommand *= SLOW_FACTOR;
            strafeCommand *= SLOW_FACTOR;
            turnCommand *= SLOW_FACTOR;
        }

        double maxPower = max(1, abs(forwardCommand) + abs(strafeCommand) + abs(turnCommand));

        drivetrain.setTeleOpDrive(
                forwardCommand / maxPower,
                strafeCommand / maxPower,
                -turnCommand / maxPower,
                false,
                isRedAlliance ? PI : 0
        );
    }

    void printTo(Telemetry telemetry) {
        Pose pose = drivetrain.getPose();
        double heading = pose.getHeading();
        telemetry.addLine("DRIVETRAIN:");
        telemetry.addLine();
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)", toDegrees(heading));
        telemetry.addData("Heading (rad)", heading);
    }


    public void startTeleopDrive() {
        drivetrain.startTeleopDrive(true);
    }
    public void setPose(Pose pose) {
        drivetrain.setPose(pose);
    }
    public Pose getPose() {
        return drivetrain.getPose();
    }
    public void update() {
        drivetrain.update();
    }

    public void setHeadingWithStick(double x, double y, boolean isRedAlliance) {
        if (x*x + y*y >= 0.64)
            drivetrain.setHeading((isRedAlliance ? -1 : 1) * PI/2 - atan2(y, x));
    }
}
