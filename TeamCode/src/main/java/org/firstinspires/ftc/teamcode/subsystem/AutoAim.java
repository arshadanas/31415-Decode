package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_FIELD;
import static org.firstinspires.ftc.teamcode.subsystem.LaunchZone.NEAR;
import static org.firstinspires.ftc.teamcode.subsystem.Shooter.LAUNCH_RAD_SHALLOWEST;
import static org.firstinspires.ftc.teamcode.subsystem.Shooter.LAUNCH_RAD_STEEPEST;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.teamcode.subsystem.utility.Profiler;

@Config
public final class AutoAim {

    public static double
            GOAL_OFFSET_Y = -0,
            GOAL_OFFSET_X = 3.5,
            RPM_NEAR = 5500,
            LAUNCH_RAD_NEAR = 1.0776000610289713,
            RPM_FAR = 6000,
            LAUNCH_RAD_FAR = 0.7853981633974483,
            TURRET_X_OFFSET = -1.86759;

    static double launchRPM, launchAngle, turretAngle;
    static boolean isRedAlliance;
    static LaunchZone currentZone;

    static void update(Pose pose, Vector velocity, double θ_, double currentRPM) {

        Vector2 G = new Vector2(
                isRedAlliance ? SIZE_FIELD - GOAL_OFFSET_X : GOAL_OFFSET_X,
                SIZE_FIELD + GOAL_OFFSET_Y
        );

        double
                k = TURRET_X_OFFSET,
                θ = normalizeRadians(pose.getHeading());

        Vector2 R = new Vector2(pose.getX(), pose.getY()); // robot center
        Vector2 R_ = new Vector2(velocity.getXComponent(), velocity.getYComponent());
        Vector2 S = new Vector2(R.x + k*cos(θ), R.y + k*sin(θ)); // shooter/turret center
        Vector2 S_ = new Vector2(R_.x - θ_*k*sin(θ), R_.y + θ_*k*cos(θ));
        Vector2 rVector = S.to(G);

        double
                m = S_.getMagnitude(),
                a = S_.getDirection(),
                r = rMag = rVector.getMagnitude(),
                b = rVector.getDirection(),
                g = b - a,
                r_ = -m*cos(g),
                h_ = m*sin(g);


        Profiler.start("aim_turret");

        turretAngle = -rVector.getAngleBetween(θ);

        Profiler.end("aim_turret");


        Profiler.start("GetCurrentZone");
        currentZone = NEAR;//LaunchZone.getCurrentZone(pose); TODO enable
        Profiler.end("GetCurrentZone");


        if (currentZone == NEAR) {
            launchRPM = RPM_NEAR;
            launchAngle = LAUNCH_RAD_NEAR;
        } else {
            launchRPM = RPM_FAR;
            launchAngle = LAUNCH_RAD_FAR;
        }


    }
    static double rMag;

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
