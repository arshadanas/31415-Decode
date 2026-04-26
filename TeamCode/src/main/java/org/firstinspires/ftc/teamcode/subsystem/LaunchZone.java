package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.LENGTH_DRIVETRAIN;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.LENGTH_INTAKE;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SIZE_FIELD;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.SIZE_TILE_DIAG;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.WIDTH_DRIVETRAIN;
import static java.lang.Math.toRadians;

import org.dyn4j.collision.narrowphase.Gjk;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Transform;

public enum LaunchZone {
    NONE,
    NEAR,
    FAR;

    private static final Gjk collisionSolver = new Gjk();

    private static final Rectangle
            nearZoneRect = new Rectangle(3 * SIZE_TILE_DIAG, 3 * SIZE_TILE_DIAG),
            farZoneRect = new Rectangle(SIZE_TILE_DIAG, SIZE_TILE_DIAG),
            robotRect = new Rectangle(LENGTH_DRIVETRAIN + LENGTH_INTAKE, WIDTH_DRIVETRAIN * 1);

    private static final Transform
            nearZonePosition = new Transform(),
            farZonePosition = new Transform(),
            robotPose = new Transform();

    static {
        nearZonePosition.rotate(toRadians(45));
        nearZonePosition.translate(SIZE_FIELD / 2, SIZE_FIELD);

        farZonePosition.rotate(toRadians(45));
        farZonePosition.translate(SIZE_FIELD / 2, 0);
    }

    static LaunchZone getCurrentZone(double x, double y, double heading) {

        robotPose.identity();
        robotPose.translate(LENGTH_INTAKE/2, 0);
        robotPose.rotate(heading);
        robotPose.translate(x, y);

        return
                collisionSolver.detect(robotRect, robotPose, nearZoneRect, nearZonePosition) ? NEAR :
                collisionSolver.detect(robotRect, robotPose, farZoneRect, farZonePosition) ? FAR :
                                                                                                                                NONE;
    }

    public static void main(String... args) {

        for (double[] pose : new double[][]{
                {103.05710814094775, 75.76184690157962, toRadians(73)}, // NONE
                {45.04407033129976, 84.21034755347013, toRadians(132)}, // NONE
                {89.888919198684, 18.023468057366358, toRadians(136)}, // NONE

                {44.480836954507055, 84.58583647133193, toRadians(169)}, // NEAR
                {123.30743288838414, 111.52020860495438, toRadians(169)}, // NEAR
                {71.67770668238674, 64.95958279009125, toRadians(180)}, // NEAR
                {106.78592050246495, 93.30899608865711, toRadians(136)}, // NEAR

                {72.42868451811033, 29.475880052151236, toRadians(180)}, // FAR
                {52.52777187143498, 14.831812255541072, toRadians(152)}, // FAR
        }) System.out.println(LaunchZone.getCurrentZone(pose[0], pose[1], pose[2]));
    }
}
