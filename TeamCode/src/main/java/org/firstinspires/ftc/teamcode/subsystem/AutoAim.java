package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_FIELD;
import static java.lang.Math.PI;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.Profiler;

@Config
public final class AutoAim {

    public static double
            GOAL_OFFSET_Y = 0,
            GOAL_OFFSET_X = 0,
            LAUNCH_RPM_TUNING = 5000,
            LAUNCH_RAD_TUNING = 1.0776000610289713;

    double launchRPM, launchAngle, turretAngle, r0, r_t, airtime;
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

        double
                heading = normalizeRadians(pose.getHeading()),
                TURRET_X_OFFSET = -1.86759;

        Vector2 s0 = new Vector2(pose.getX(), pose.getY())
                        .sum(Vector2.create(TURRET_X_OFFSET, heading));
        Vector2 v0 = new Vector2(velocity.getXComponent(), velocity.getYComponent())
                        .sum(Vector2.create(angVel * TURRET_X_OFFSET, heading + PI / 2));

        Vector2 launchVec = s0.to(G);
        r0 = launchVec.getMagnitude();

        Profiler.start("iterate airtime");
        airtime = getFinalAirtime(s0, v0, G);
        Profiler.end("iterate airtime");

        Vector2 s_t = s0.sum(v0.product(airtime));
        Vector2 launchVec_t = s_t.to(G);
        r_t = launchVec_t.getMagnitude();

        turretAngle = -launchVec_t.getAngleBetween(heading);
        launchRPM = 21.27491 * r_t + 2916.29066;
        launchAngle = -0.00307104 * r_t + 1.22222;
//        launchRPM = LAUNCH_RPM_TUNING;
//        launchAngle = LAUNCH_RAD_TUNING;

    }

    /**
     * <a href="https://www.desmos.com/calculator/9rno9gfxn7">Desmos</a>
     */
    private static double getFinalAirtime(Vector2 s0, Vector2 v0, Vector2 G) {
        double airtime = 0;
        int iterations = 15;
        for (int i = 0; i < iterations; i++) {
            Vector2 s_t = s0.sum(v0.product(airtime));
            airtime = getAirtime(s_t.distance(G));
        }
        return airtime;
    }

    // TODO curve fit, determine # iterations, & clip output
    private static double getAirtime(double distToGoal) {
        return 0;
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

    static double getRPMDrop(double preLaunchRPM) {
        return 0.271632 * preLaunchRPM + 109.1459;
    }

    public static void main(String[] args) {

        Vector2 ad = new Vector2();
        double sum = 0;

        for (int i = 0; i < 200; i++){
            double a = System.nanoTime();
            double airtime = getFinalAirtime(
                    new Vector2(-70.75, -70.75),     // S_0
                    new Vector2(-63.4788154, -63.8), // S_vel_0
                    new Vector2(70.75, 70.75)        // G
            );
            sum += ((System.nanoTime() - a) / 1e+6);
        }

        System.out.println(sum / 200);
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("Current zone", currentZone);
        telemetry.addLine();
        telemetry.addData("Shooter-goal dist at 0 (in)", r0);
        telemetry.addData("Shooter-goal dist at t (in)", r_t);
        telemetry.addLine();
        telemetry.addData("Turret angle at 0 (deg)", toDegrees(turretAngle));
        telemetry.addLine();
        telemetry.addData("Computed airtime t", airtime);
    }
}
