package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_FIELD;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.teamcode.subsystem.utility.Profiler;

@Config
public final class AutoAim {

    public static double
            GOAL_OFFSET_Y = -5,
            GOAL_OFFSET_X = 3.5,
            LAUNCH_RPM_NEAR = 5000,
            LAUNCH_RAD_NEAR = 1.0776000610289713,
            LAUNCH_RPM_FAR = 6000,
            LAUNCH_RAD_FAR = 0.7853981633974483,
            TURRET_X_OFFSET = -1.86759,
            CURVE_FIT_RPM_MIN = 2916.29066,
            CURVE_FIT_RPM_SLOPE = 21.27491,
            CURVE_FIT_ANGLE_SLOPE = -0.00307104,
            CURVE_FIT_ANGLE_Y_INT = 1.22222;

    double launchRPM, launchAngle, turretAngle, r;
    LaunchZone currentZone;

    private Vector2 G;

    AutoAim() {}

    void setAlliance(boolean isRedAlliance) {
        G = new Vector2(
                isRedAlliance ? SIZE_FIELD - GOAL_OFFSET_X : GOAL_OFFSET_X,
                SIZE_FIELD + GOAL_OFFSET_Y
        );
    }

    void update(Pose pose, Vector velocity, double θ_, double currentRPM) {

        Profiler.start("GetCurrentZone");
        currentZone = LaunchZone.getCurrentZone(pose);
        Profiler.end("GetCurrentZone");

        double
                k = TURRET_X_OFFSET,
                θ = normalizeRadians(pose.getHeading());

        Vector2 R = new Vector2(pose.getX(), pose.getY()); // robot center
        Vector2 R_ = new Vector2(velocity.getXComponent(), velocity.getYComponent());
        Vector2 S = new Vector2(R.x + k*cos(θ), R.y + k*sin(θ)); // shooter/turret center
        Vector2 S_ = new Vector2(R_.x - θ_*k*sin(θ), R_.y + θ_*k*cos(θ));
        Vector2 rVector = S.to(G);
        r = rVector.getMagnitude();

        double
                m = S_.getMagnitude(),
                a = S_.getDirection(),
                b = rVector.getDirection(),
                g = b - a,
                r_ = -m*cos(g),
                h_ = m*sin(g);


        Profiler.start("aim_turret");

        turretAngle = -rVector.getAngleBetween(θ);

        Profiler.end("aim_turret");

        launchRPM = CURVE_FIT_RPM_SLOPE * r + CURVE_FIT_RPM_MIN;
        launchAngle = CURVE_FIT_ANGLE_SLOPE * r + CURVE_FIT_ANGLE_Y_INT;

    }

    /**
     * Inverse of {@link #getLaunchVel}
     * @return Angular velocity of shooter wheel at launch, in rpm
     */
    private static double getLaunchRPM(double inPerSec) {
        return 29.68064 * inPerSec - 0.445157; // TODO Tune empirically
    }

    /**
     * Inverse of {@link #getLaunchRPM}
     * @return Linear velocity of {@link Artifact} at launch, in inches/sec
     */
    private static double getLaunchVel(double rpm) {
        return (rpm + 0.445157) / 29.68064; // TODO Tune empirically
    }

    private static double getRPMDrop(double preLaunchRPM) {
        return 0.271632 * preLaunchRPM + 109.1459;
    }
}
