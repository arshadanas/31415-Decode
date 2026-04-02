package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Math.PI;
import static java.lang.Math.acos;
import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.cbrt;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.teamcode.control.Ranges;

/**
 * Solution to the circular-parabolic fused-arc kinematic shooting model for the FTC DECODE season <br>
 * Displacements are in inches (in) <br>
 * Angles are in radians (rad) <br>
 * Durations are in seconds (s) <br>
 *
 * @implSpec <a href="https://www.desmos.com/calculator/rq5t2y3n3f">Desmos</a>
 *
 * @author Arshad Anas
 * @since 2025/09/18
 */
@Config
public final class KinematicsSolver {

    public static final Vector2 // tunable
            o_goal = new Vector2(5, -2);

    private static final double
            a_G = -386.0886,
            θ_launchMin = toRadians(31.901328),
            θ_launchMax = toRadians(61.7419355),
            θ_avg = (θ_launchMin + θ_launchMax) / 2,
            v_launchMin = 177,
            v_launchMax = 350,
            o_turretForward = -1.86759,
            y_goal = 40,
            y_rim = 38.75,
            r_rimClearance = 0.75,
            r_ball = 2.5,
            r_compression = 4/25.4,
            r_wheel_physical = 1.5,
            r_wheel = r_wheel_physical - r_compression,
            c = r_wheel + r_ball,
            half_F = 141.5/2;

    private static final Vector2
            s_wheel = new Vector2(3.560949685039,10.4596456693),
            Center = new Vector2(half_F, half_F);

    private final Vector2
            // constant after init
            G = new Vector2(),

            // constant during solving
            s_turret = new Vector2(),
            v_turret = new Vector2(),
            unitTurretToGoal = new Vector2(),

            // modified during solving
            s0 = new Vector2(),
            s_launch = new Vector2(),
            unitLaunchPtToGoal = new Vector2(),
            v_relToGoal = new Vector2(),
            k = new Vector2(),
            s_goal = new Vector2(),
            s_rim = new Vector2(),
            v0 = new Vector2(),
            s_rimNearest = new Vector2(),
            s_rimTarget = new Vector2();

    private double m2, b;

    double θ_launch = θ_avg, v_launch = 275, α_launch = 0;

    public void setAlliance(boolean isRedAlliance) {
        double i = isRedAlliance ? 1 : -1;

        G.set(Center).add((half_F - o_goal.x) * i, half_F + o_goal.y);

        Vector2
                A_rim = Center.sum(47.42197 * i, 70.04449),
                B_rim = A_rim.sum(15.85266 * i, -21.85619);

        m2 = (B_rim.y - A_rim.y) / (B_rim.x - A_rim.x);
        b = -m2 * A_rim.x + A_rim.y;
    }

    public void setRobotState(Pose pose, Vector velocity, double angVel) {
        double
                heading = pose.getHeading(),
                turretX = o_turretForward * cos(heading),
                turretY = o_turretForward * sin(heading);
        s_turret.set(pose.getX() + turretX, pose.getY() + turretY);
        v_turret.set(velocity.getXComponent() + angVel * -turretY, velocity.getYComponent() + angVel * turretX);
        unitTurretToGoal.set(G);
        unitTurretToGoal.subtract(s_turret);
        unitTurretToGoal.normalize();
//        System.out.println("s_turret: " + s_turret);
//        System.out.println("v_turret: " + v_turret);
//        System.out.println("unitTurretToGoal: " + unitTurretToGoal);
//        System.out.println();
    }

    private void setRobotState(double x, double y, double heading, double vx, double vy, double angVel) {
        Vector vRobot = new Vector();
        vRobot.setOrthogonalComponents(vx, vy);
        setRobotState(new Pose(x, y, heading), vRobot, angVel);
    }

    private void computeForwardKinematics() {
        double cos_θ_launch = cos(θ_launch), sin_θ_launch = sin(θ_launch);

        s0.set(s_wheel).add(-c*sin_θ_launch, c*cos_θ_launch);

        s_launch.set(unitTurretToGoal).rotate(α_launch).multiply(s0.x).add(s_turret);
        unitLaunchPtToGoal.set(G).subtract(s_launch).normalize();
        v_relToGoal.set(
                -v_turret.dot(unitLaunchPtToGoal),
                -v_turret.dot(-unitLaunchPtToGoal.y, unitLaunchPtToGoal.x*1)
        );

        double m1 = (G.y - s_launch.y) / (G.x - s_launch.x);
        double k_x = (m1 * s_launch.x - s_launch.y + b) / (m1 - m2);
        k.set(k_x, m2 * k_x + b);

        s_goal.set(s_launch.distance(G) + s0.x, y_goal);
        s_rim.set(s_launch.distance(k) + s0.x, y_rim);

        v0.set(
                sqrt(v_launch*v_launch * cos_θ_launch*cos_θ_launch - v_relToGoal.y*v_relToGoal.y) - v_relToGoal.x,
                v_launch * sin_θ_launch
        );

//        System.out.println("s0: " + s0);
//        System.out.println("s_launch: " + s_launch);
//        System.out.println("unitLaunchPtToGoal: " + unitLaunchPtToGoal);
//        System.out.println("v_relToGoal: " + v_relToGoal);
//        System.out.println("k: " + k);
//        System.out.println("s_goal: " + s_goal);
//        System.out.println("s_rim: " + s_rim);
//        System.out.println("v0: " + v0);
    }

    private static double W(double x, double A, double B, double C, double D) {
        return (((A/4*x + B/3)*x + C/2)*x + D)*x;
    }

    private void computeRimApproach(double vi, double tan_θi) {

        double
                M = tan_θi,
                N = (2*a_G / (vi*vi)) * (1 + M*M),
                O = 2*M - N*s0.x,
                P = s_rim.y - s0.y + 0.25*s0.x*(2*M + O),

                A = 0.25*N*N,
                B = 0.75*N*O,
                C = 2 + 0.5*O*O - N*P,
                D = -2*s_rim.x - O*P,

                b3a = B / (3*A),
                Q = C/(3*A) - b3a*b3a,
                R = (b3a*C - D) / (2*A) - b3a*b3a*b3a,
                Q3 = Q*Q*Q,
                Dc = Q3 + R*R,

                x_nearest;

        if (Dc < 0) {
            double
                    theta = acos( R / sqrt(-Q3) ) / 3,
                    sQ = 2 * sqrt(-Q),
                    twoThirdsPi = 2 * PI / 3,
                    x1 = sQ * cos(theta) - b3a,
                    x2 = sQ * cos(theta + twoThirdsPi) - b3a,
                    x3 = sQ * cos(theta + 2*twoThirdsPi) - b3a,
                    d1 = W(x1, A, B, C, D),
                    d2 = W(x2, A, B, C, D),
                    d3 = W(x3, A, B, C, D);

            x_nearest = (d1 < d2) ?
                            (d1 < d3) ? x1 : x3 :
                            (d2 < d3) ? x2 : x3;

        } else if (Dc > 0) {
            double
                    sqrtD = sqrt(Dc),
                    S = cbrt(R + sqrtD),
                    T = cbrt(R - sqrtD);

            x_nearest = (S + T) - b3a;
        } else {
            double
                    cbrtR = cbrt(R),
                    x1 = 2*cbrtR - b3a,
                    x2 =   cbrtR - b3a,
                    d1 = W(x1, A, B, C, D),
                    d2 = W(x2, A, B, C, D);

            x_nearest = (d1 < d2) ? x1 : x2;
        }

        double dx = x_nearest - s0.x;
        s_rimNearest.set(x_nearest, 0.25*N*dx*dx + M*dx + s0.y);

        s_rimTarget.set(s_rimNearest).subtract(s_rim).setMagnitude(r_ball + r_rimClearance);
        if (s_rimTarget.y < 0)
            s_rimTarget.negate();
        s_rimTarget.add(s_rim);

    }

    private double v1(double cos_θ, double tan_θ) {
        double dx = s_goal.x - s0.x;
        return dx / ( cos_θ * sqrt( (2 / a_G) * (s_goal.y - s0.y - dx*tan_θ) ));
    }

    private double θ_3pt(Vector2 target, double y_offset) {
        double
                dx = s_goal.x - s0.x,
                dy = s_goal.y - s0.y,
                tx = target.x - s0.x,
                ty = target.y - s0.y + y_offset;

        return atan(
                ( dx*dx*ty - tx*tx*dy ) /
                ( dx*tx*(dx - tx)  )
        );
    }

    public void generateTarget_v_θ_α() {
//        long a = System.nanoTime();
        θ_launch = θ_avg;
        α_launch = 0;
        double θi, cos_θi, sin_θi, tan_θi = 0, vi = 0, vf, θf, α;
//        System.out.println(θ_avg);

        for (int j = 0; j < 5; j++) {
            // begin iteration
            computeForwardKinematics();

            if (j == 0) {
                θi = θ_3pt(s_rim, r_ball + r_rimClearance);
                tan_θi = sin(θi) / (cos_θi = cos(θi));
                vi = v1(cos_θi, tan_θi);
//                System.out.println("init guess");
//                System.out.println("θi: " + θi);
//                System.out.println("vi: " + vi);
            }

            // start iteration
            computeRimApproach(vi, tan_θi);
            θi = θ_3pt(s_rimTarget, 0);
//            System.out.println("s_rimNearest: " + s_rimNearest);
//            System.out.println("s_rimTarget: " + s_rimTarget);
//            System.out.println("θi: " + θi);

            int n = 3;
            for (int i = 0; i < n; i++) {
                tan_θi = (sin_θi = sin(θi)) / (cos_θi = cos(θi));
                vi = v1(cos_θi, tan_θi);

                vf = sqrt(vi * vi + v_relToGoal.getMagnitudeSquared() + 2 * v_relToGoal.x * vi * cos_θi);
                θf = asin(vi * sin_θi / vf);
                α = atan(v_relToGoal.y / (vi * cos_θi + v_relToGoal.x));

//                System.out.println("θf: " + θf);
//                System.out.println("vf: " + vf);
//                System.out.println("vi: " + vi);
//                System.out.println("α: " + α);

                if (i + 1 >= n || θ_launchMin <= θf && θf <= θ_launchMax) {
                    θ_launch = θf;
                    v_launch = vf;
                    α_launch = α;
//                    System.out.println("θ_launch: " + θ_launch);
//                    System.out.println("v_launch: " + v_launch);
//                    System.out.println("α_launch: " + α_launch);
                    break;
                }

                double θ_clipped = Ranges.clip(θf, θ_launchMin, θ_launchMax);
                double cos_clipped = cos(θ_clipped), sin_clipped = sin(θ_clipped);
                θi = atan(
                        vf * sin_clipped /
                        ( sqrt(vf * vf * cos_clipped * cos_clipped - v_relToGoal.y * v_relToGoal.y) - v_relToGoal.x )
                );
//                System.out.println("θ_clipped: " + θ_clipped);
//                System.out.println("θi post clip: " + θi);
//                System.out.println();
            }
//            System.out.println();
//            System.out.println();
        }
//        b(a);
    }

    public String resultsToString() {
        return "θ_launch: " + θ_launch + "\nv_launch: " + v_launch + "\nα_launch: " + α_launch;
    }

    public static void main(String[] args) {
        new Vector2(3,3).normalize();
        new Pose(0, 0, 0);

        KinematicsSolver solver = new KinematicsSolver();
        solver.setAlliance(true);

        solver.setRobotState(71.5,70, -3.12, 0.5, 95, 0.13);
        solver.generateTarget_v_θ_α();
        System.out.println(solver.resultsToString());
        System.out.println();

        a();
        solver.setRobotState(40.9,102, -1.46, 61.5, -6.2, 0.13);
        solver.generateTarget_v_θ_α();
        System.out.println(solver.resultsToString());
        b();
        System.out.println();

        a();
        solver.setRobotState(111.5,120, -1.46, -14.3, -33.5, 0.13);
        solver.generateTarget_v_θ_α();
        System.out.println(solver.resultsToString());
        b();
        System.out.println();
    }

    private static long a;
    private static void a() {
        a = System.nanoTime();
    }
    private static void b() {
        System.out.println((System.nanoTime() - a) * 1e-6);
    }

}
