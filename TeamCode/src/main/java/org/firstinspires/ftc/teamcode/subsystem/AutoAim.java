package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_FIELD;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.Profiler;

@Config
public final class AutoAim {

    public static Vector2 GOAL_OFFSET = new Vector2(0, 0);

    public static double
            LAUNCH_RPM_TUNING = 5000,
            LAUNCH_RAD_TUNING = 1.0776000610289713;

    double launchRPM, launchAngle, turretAngle, r0, r_t, airtime;
    LaunchZone currentZone;

    private final Vector2
            G = new Vector2(0, SIZE_FIELD + GOAL_OFFSET.y),
            launchVec = new Vector2();

    void setAlliance(boolean isRedAlliance) {
        G.x = isRedAlliance ? SIZE_FIELD - GOAL_OFFSET.x : GOAL_OFFSET.x;
    }

    void update(Pose pose, Vector velocity, double angVel, double currentRPM) {

        Profiler.start("GetCurrentZone");
        currentZone = LaunchZone.getCurrentZone(pose);
        Profiler.end("GetCurrentZone");

        double
                heading = normalizeRadians(pose.getHeading()),
                TURRET_X_OFFSET = -1.86759;

        Vector2 s0 = Vector2.create(TURRET_X_OFFSET, heading)
                            .add(pose.getX(), pose.getY());

        Vector2 v0 = Vector2.create(angVel * TURRET_X_OFFSET, heading + PI / 2)
                            .add(velocity.getXComponent(), velocity.getYComponent());

        launchVec.set(G).subtract(s0); // L = G - s0
        r0 = launchVec.getMagnitude();

        Profiler.start("iterate airtime");
        airtime = getFinalAirtime(launchVec, v0);
        Profiler.end("iterate airtime");

        launchVec.subtract(v0.product(airtime)); // L -= v0*t

        r_t = launchVec.getMagnitude();
        turretAngle = -launchVec.getAngleBetween(heading);
        launchRPM = 21.27491 * r_t + 2916.29066;
        launchAngle = -0.00307104 * r_t + 1.22222;
//        launchRPM = LAUNCH_RPM_TUNING;
//        launchAngle = LAUNCH_RAD_TUNING;

    }

    /**
     * <a href="https://www.desmos.com/calculator/9rno9gfxn7">Desmos</a>
     */
    private static double getFinalAirtime(Vector2 launchVec, Vector2 v0) {
        double airtime = getLinearAirtime(launchVec, v0);
        int iterations = 15;
        for (int i = 0; i < iterations; i++)
            airtime = getAirtime(v0.product(airtime).negate().add(launchVec).getMagnitude()); // |L - v0*t|
        return airtime;
    }

    /**
     * @return closed form airtime solution for a linear airtime curve-fit
     */
    private static double getLinearAirtime(Vector2 launchVec, Vector2 v0) {
        double // Mx + B
                M = 0.00343072843338, // TODO curve fit
                B = 0.286480431595,
                M_invSquared = 1 / (M*M),
                // quadratic coeffs
                a = v0.getMagnitudeSquared() - M_invSquared,
                b = -2 * v0.dot(launchVec) + 2*B*M_invSquared,
                c = launchVec.getMagnitudeSquared() - B*B*M_invSquared;

        return 0;
//        return ( -b - sqrt(b*b - 4*a*c) ) / (2*a);
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
        int n = 200;
        double a = System.nanoTime();

        Vector2 s0 = new Vector2(-70.75, -70.75);
        Vector2 v0 = new Vector2(-63.4788154, -63.8);
        Vector2 G = new Vector2(70.75, 70.75);

        for (int i = 0; i < n; i++)
            getFinalAirtime(s0.to(G), v0);

        System.out.println((System.nanoTime() - a) / n / 1e+6);
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("Current zone", currentZone);
        telemetry.addLine();
        telemetry.addData("Shooter-goal dist at 0 (in)", r0);
        telemetry.addData("Shooter-goal dist at t (in)", r_t);
        telemetry.addData("Computed airtime t (s)", airtime);
    }
}
