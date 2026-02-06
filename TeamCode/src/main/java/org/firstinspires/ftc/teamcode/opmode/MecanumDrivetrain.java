package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;


public final class MecanumDrivetrain {

    private final MecanumDrive mecanumDrivetrain;

    private final MotorEx[] motors;

    public final GoBildaPinpointDriver pinpoint;

    public MecanumDrivetrain(HardwareMap hardwareMap) {

        // Assign motors using their hardware map names, each drive-type can have different names if needed
        motors = new MotorEx[]{
                new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_435),
        };

        motors[0].setInverted(true);
        motors[1].setInverted(false);
        motors[2].setInverted(true);
        motors[3].setInverted(false);

        for (MotorEx motor : motors) motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Initialize the FTCLib drive-base
        mecanumDrivetrain = new MecanumDrive(false, motors[0], motors[1], motors[2], motors[3]);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(Constants.pinpointConstants.forwardPodY, Constants.pinpointConstants.strafePodX, DistanceUnit.INCH);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(Constants.pinpointConstants.forwardEncoderDirection, Constants.pinpointConstants.strafeEncoderDirection);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    public void run(double xCommand, double yCommand, double turnCommand) {
        // normalize inputs
        double max = Math.max(xCommand + yCommand + turnCommand, 1.0);
        xCommand /= max;
        yCommand /= max;
        turnCommand /= max;

        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);

        mecanumDrivetrain.driveFieldCentric(xCommand, yCommand, turnCommand, heading);
    }
}
