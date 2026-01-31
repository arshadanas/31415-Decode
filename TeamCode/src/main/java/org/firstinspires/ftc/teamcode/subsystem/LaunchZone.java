package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_FIELD;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_TILE;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.pedropathing.geometry.Pose;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.util.GeometricShapeFactory;

public enum LaunchZone {
    NONE,
    NEAR,
    FAR;

    private static final double
            FORWARD_OFFSET_IN = 1.45714,
            LENGTH_TOTAL_IN = 17.73172,
            WIDTH_TOTAL_IN = 15.53937;

    private final static GeometricShapeFactory factory = new GeometricShapeFactory();
    private final static Polygon nearTriangle, farTriangle;

    static {
        GeometryFactory geometryFactory = new GeometryFactory();

        Coordinate[] near = new Coordinate[]{
                new Coordinate(0, SIZE_FIELD),
                new Coordinate(SIZE_FIELD, SIZE_FIELD),
                new Coordinate(SIZE_FIELD / 2, SIZE_FIELD / 2)
        };
        nearTriangle = geometryFactory.createPolygon(near);

        Coordinate[] far = new Coordinate[]{
                new Coordinate(SIZE_TILE * 2, 0),
                new Coordinate(SIZE_TILE * 3, SIZE_TILE),
                new Coordinate(SIZE_TILE * 4, 0)
        };

        farTriangle = geometryFactory.createPolygon(far);

    }

    static LaunchZone getCurrentZone(Pose currentPose) {

        double heading = currentPose.getHeading();
        factory.setCentre(new Coordinate(
                currentPose.getX() + (FORWARD_OFFSET_IN * cos(heading)),
                currentPose.getY() + (FORWARD_OFFSET_IN * sin(heading))
        ));
        // 2d width corresponds to robot length when heading = 0
        factory.setWidth(LENGTH_TOTAL_IN);
        // 2d height corresponds to robot width when heading = 0
        factory.setHeight(WIDTH_TOTAL_IN * 1);

        factory.setRotation(heading);

        Polygon rectangle = factory.createRectangle();


        return
                rectangle.intersects(nearTriangle) ? NEAR :
                rectangle.intersects(farTriangle) ?  FAR :
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
