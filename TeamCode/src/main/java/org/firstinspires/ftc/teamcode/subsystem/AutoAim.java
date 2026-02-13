package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_FIELD;
import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.max;
import static java.lang.Math.round;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.Profiler;
import org.firstinspires.ftc.teamcode.control.Ranges;

@Config
public final class AutoAim {

    public static double
            GOAL_OFFSET_Y = 0,
            GOAL_OFFSET_X = 0,
            LAUNCH_RPM = 5000,
            LAUNCH_RAD = 1.0776000610289713,
            TURRET_X_OFFSET = -1.86759,
            CURVE_FIT_RPM_MIN = 2916.29066,
            CURVE_FIT_RPM_SLOPE = 21.27491,
            CURVE_FIT_ANGLE_SLOPE = -0.00307104,
            CURVE_FIT_ANGLE_Y_INT = 1.22222;

    double launchRPM, launchAngle, turretAngle, r, r_t, airtime;
    LaunchZone currentZone;

    private Vector2 G;

    AutoAim() {}

    void setAlliance(boolean isRedAlliance) {
        G = new Vector2(
                isRedAlliance ? SIZE_FIELD - GOAL_OFFSET_X : GOAL_OFFSET_X,
                SIZE_FIELD + GOAL_OFFSET_Y
        );
    }

    void update(Pose pose, Vector velocity, double angVel, double currentRPM) {

        Profiler.start("GetCurrentZone");
        currentZone = LaunchZone.getCurrentZone(pose);
        Profiler.end("GetCurrentZone");

        double heading = normalizeRadians(pose.getHeading());

        Vector2 R = new Vector2(pose.getX(), pose.getY()); // robot center
        Vector2 R_vel = new Vector2(velocity.getXComponent(), velocity.getYComponent());

        Vector2 S = getTurretCenter(R, heading);
        Vector2 S_vel = getTurretVel(R_vel, heading, angVel);

        Vector2 rVector = S.to(G);
        r = rVector.getMagnitude();

        double
                m = S_vel.getMagnitude(),
                a = S_vel.getDirection(),
                b = rVector.getDirection(),
                g = b - a,
                r_vel = -m*cos(g),
                h_vel = m*sin(g);

        Profiler.start("iterate airtime");
        airtime = getFinalAirtime(S, S_vel, G);
        Profiler.end("iterate airtime");

        Vector2 S_t = S.sum(S_vel.product(airtime));
        Vector2 rVector_t = S_t.to(G);
        r_t = rVector_t.getMagnitude();

        Profiler.start("aim_turret");

        turretAngle = -rVector_t.getAngleBetween(heading);

        Profiler.end("aim_turret");

        launchRPM = CURVE_FIT_RPM_SLOPE * r_t + CURVE_FIT_RPM_MIN;
        launchAngle = CURVE_FIT_ANGLE_SLOPE * r_t + CURVE_FIT_ANGLE_Y_INT;
//        launchRPM = LAUNCH_RPM;
//        launchAngle = LAUNCH_RAD;

    }

    private static Vector2 getTurretCenter(Vector2 R, double heading) {
        return R.sum(Vector2.create(TURRET_X_OFFSET, heading));
    }

    private static Vector2 getTurretVel(Vector2 R_vel, double heading, double angVel) {
        return R_vel.sum(Vector2.create(angVel * TURRET_X_OFFSET, heading + PI/2));
    }

    private static double getFinalAirtime(Vector2 S_0, Vector2 S_vel_0, Vector2 G) {
        double airtime = 0;
        int iterations = 7; // TODO vary iterations based on |S_vel_0|
        for (int i = 0; i < iterations; i++) {
            Vector2 S_t = S_0.sum(S_vel_0.product(airtime));
            double distToGoal = S_t.distance(G);
            airtime = 0; // TODO curve fit airtime
        }
        return Ranges.clip(airtime, 0, 3); // airtime in [0,3] seconds
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

    public static void main(String[] args) {

        Vector2 G = new Vector2(70.75,70.75);
        Vector2 R_0 = new Vector2(-7.1,0.4);
        double heading_0 = 0;
        Vector2 R_vel_0 = new Vector2(44.8,26.4);
        double angVel_0 = 0;

        double old = TURRET_X_OFFSET;
        TURRET_X_OFFSET = 0;
        Vector2 S_0 = getTurretCenter(R_0, heading_0);
        Vector2 S_vel_0 = getTurretVel(R_vel_0, heading_0, angVel_0);
        TURRET_X_OFFSET = old;

        double a = System.nanoTime();
        double airtime = getFinalAirtime(S_0, S_vel_0, G);
        System.out.println((System.nanoTime() - a)/1e+6);
        System.out.println(airtime);
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("Current zone", currentZone);
        telemetry.addData("Shooter-goal dist at 0 (in)", r);
        telemetry.addData("Shooter-goal dist at t (in)", r_t);
        telemetry.addData("Computed airtime t", airtime);
    }
}
