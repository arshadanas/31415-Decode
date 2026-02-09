package org.firstinspires.ftc.teamcode.pedropathing;

import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED;
import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrivetrain;

public class Constants {

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(-110.5/25.4)
            .strafePodX(-108.53456/25.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(goBILDA_4_BAR_POD)
            .forwardEncoderDirection(REVERSED)
            .strafeEncoderDirection(FORWARD)
    ;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(16.782904)
            .forwardZeroPowerAcceleration(-29.66287519)
            .lateralZeroPowerAcceleration(-59.382531949)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.025, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0.0,0.0005,0.6,0.05))
            .centripetalScaling(0.0005)
    ;

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .motorCachingThreshold(MecanumDrivetrain.MOTOR_CACHING_THRESHOLD)
            .xVelocity(74.66360786)
            .yVelocity(55.132251168)
    ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}
