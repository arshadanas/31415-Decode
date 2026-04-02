package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_FIELD;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_TILE;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.pedropathing.geometry.Pose;

import org.dyn4j.collision.narrowphase.Gjk;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Transform;
import org.firstinspires.ftc.teamcode.opmode.Auto;

public enum LaunchZone {
    NONE,
    NEAR,
    FAR;

    private static final Transform nearZonePosition, farZonePosition, robotPose = new Transform();
    private static final Rectangle nearZoneRect, farZoneRect, robotRect;
    private static final Gjk collisionSolver = new Gjk();

    static {
        double nearZoneSize = SIZE_TILE * Math.sqrt(2) * 3;
        double farZoneSize = SIZE_TILE * Math.sqrt(2);

        nearZoneRect = new Rectangle(nearZoneSize, nearZoneSize);
        farZoneRect = new Rectangle(farZoneSize, farZoneSize);

        robotRect = new Rectangle(17.73172, Auto.WIDTH_DRIVETRAIN * 1);

        nearZonePosition = new Transform();
        farZonePosition = new Transform();

        nearZonePosition.rotate(toRadians(45));
        nearZonePosition.translate(SIZE_FIELD / 2, SIZE_FIELD);

        farZonePosition.rotate(toRadians(45));
        farZonePosition.translate(SIZE_FIELD / 2, 0);
    }

    static LaunchZone getCurrentZone(Pose currentPose) {

        double heading = currentPose.getHeading();
        robotPose.identity();
        robotPose.rotate(heading);
        double FORWARD_OFFSET = 1.45714;
        robotPose.translate(
                currentPose.getX() + (FORWARD_OFFSET * cos(heading)),
                currentPose.getY() + (FORWARD_OFFSET * sin(heading))
        );

        return
                collisionSolver.detect(robotRect, robotPose, nearZoneRect, nearZonePosition) ? NEAR :
                collisionSolver.detect(robotRect, robotPose, farZoneRect, farZonePosition) ? FAR :
                                                                                                                                NONE;
    }

    public static void main(String... args) {

        Pose[] poses = {
                new Pose(103.05710814094775, 75.76184690157962, toRadians(73)), // NONE
                new Pose(45.04407033129976, 84.21034755347013, toRadians(132)), // NONE

                new Pose(44.480836954507055, 84.58583647133193, toRadians(169)), // NEAR
                new Pose(123.30743288838414, 111.52020860495438, toRadians(169)), // NEAR
                new Pose(71.67770668238674, 64.95958279009125, toRadians(180)), // NEAR
                new Pose(106.78592050246495, 93.30899608865711, toRadians(136)), // NEAR

                new Pose(72.42868451811033, 29.475880052151236, toRadians(180)), // FAR
                new Pose(52.52777187143498, 14.831812255541072, toRadians(152)), // FAR

                new Pose(89.888919198684, 18.023468057366358, toRadians(136)), // NONE
        };

        for (Pose pose : poses)
            System.out.println(LaunchZone.getCurrentZone(pose));
    }
}
