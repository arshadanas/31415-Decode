package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_FIELD;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_TILE;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

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
}
