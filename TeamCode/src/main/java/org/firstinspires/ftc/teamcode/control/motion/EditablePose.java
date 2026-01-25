package org.firstinspires.ftc.teamcode.control.motion;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public final class EditablePose {

    public double x, y, heading;

    public EditablePose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public EditablePose(Pose pose) {
        this(pose.getX(), pose.getY(), pose.getHeading());
    }

    public EditablePose clone() {
        return new EditablePose(x, y, heading);
    }

    public Pose toPose() {
        return new Pose(x, y, heading);
    }

    public Vector toVector() {
        return new Vector(toPose());
    }

    public double angleTo(EditablePose target) {
        return atan2(target.y - this.y, target.x - this.x);
    }

    public double distTo(EditablePose target) {
        return hypot(target.y - this.y, target.x - this.x);
    }

    public String toString() {
        return x + ", " + y + ", " + toDegrees(heading) + " deg";
    }
}
