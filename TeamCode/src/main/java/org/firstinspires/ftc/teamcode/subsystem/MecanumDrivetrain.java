package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Config
public final class MecanumDrivetrain {

    public static double SLOW_FACTOR = 0.3;

    private final Follower drivetrain;

    MecanumDrivetrain(HardwareMap hardwareMap, Pose pose) {
        drivetrain = Constants.createFollower(hardwareMap);
        try {
            drivetrain.poseTracker.resetIMU();
        } catch (InterruptedException ignored) {}

        drivetrain.setStartingPose(pose);

        update();
    }

    /**
     * Drive robot with control stick inputs
     *
     * @param xCommand    positive = strafing right
     * @param yCommand    positive = forward
     * @param turnCommand positive = clockwise
     * @param useSlowMode drives robot at {@link #SLOW_FACTOR} of full speed
     */
    public void run(double xCommand, double yCommand, double turnCommand, boolean useSlowMode) {

        if (useSlowMode) {
            yCommand *= SLOW_FACTOR;
            xCommand *= SLOW_FACTOR;
            turnCommand *= SLOW_FACTOR;
        }

        drivetrain.setTeleOpDrive(
                yCommand,
                xCommand,
                turnCommand,
                true,
                isRedAlliance ? 0 : PI
        );
    }

    boolean isRedAlliance = false;

    public void printTelemetry(Telemetry telemetry) {
        Pose pose = drivetrain.getPose();
        double heading = pose.getHeading();
        telemetry.addLine("DRIVETRAIN:");
        telemetry.addLine();
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (rad)", heading);
        telemetry.addData("Heading (deg)", toDegrees(heading));
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

    public void setHeadingWithStick(double x, double y) {
        if (x*x + y*y >= 0.64)
            drivetrain.setHeading(PI / 2 - atan2(y, x));
    }
}
