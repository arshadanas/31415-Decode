package org.firstinspires.ftc.teamcode.pedropathing;

import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED;
import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

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
