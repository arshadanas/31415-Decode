package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.LaunchZone.NONE;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;

@Config
public final class Robot {

    public final MecanumDrivetrain drivetrain;
    public final Handler handler;
    public final Flywheel flywheel;
    private final Hood hood;
    public final Turret turret;

    private final BulkReader bulkReader;
    private final KinematicsSolver solver;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private LaunchZone currentZone;

    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = new MecanumDrivetrain(hardwareMap, startPose);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
        handler = new Handler(hardwareMap);
        turret = new Turret(hardwareMap);

        bulkReader = new BulkReader(hardwareMap);
        solver = new KinematicsSolver();
    }

    public void setAlliance(boolean isRedAlliance) {
        solver.setAlliance(isRedAlliance);
    }

    public void run(boolean feed, boolean forceFeed) {
        bulkReader.bulkRead();
        drivetrain.update();

        Pose pose = drivetrain.getPose();
        Vector velocity = drivetrain.getVelocity();
        double x = pose.getX(), y = pose.getY(), heading = pose.getHeading();

        currentZone = LaunchZone.getCurrentZone(x, y, heading);

        solver.setRobotState(x, y, heading, velocity.getXComponent(), velocity.getYComponent(), drivetrain.getAngularVel());
        boolean validSolve = solver.calculateTarget_v_θ_α();

        turret.setTarget(solver.getTurretAngle());
        flywheel.setVelocity(solver.v_launch);
        hood.setLaunchAngle(solver.θ_launch);

        boolean inLaunchZone = currentZone != NONE;
        boolean feedsPending = handler.feedsPending();

        flywheel.run(inLaunchZone, feedsPending);
        turret.run(feedsPending);

        boolean inTolerance = validSolve && flywheel.inTolerance() && turret.inTolerance();

        handler.run(inLaunchZone && (forceFeed || (feed && inTolerance)));
    }

    public void printTo(Telemetry telemetry) {
        telemetry.addData("LOOP TIME (ms)", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.addLine("\n--------------------------------------\n");
        telemetry.addData("Current zone", currentZone);
        telemetry.addLine();
        telemetry.addLine(solver.resultsToString());
        telemetry.addLine("\n--------------------------------------\n");
        drivetrain.printTo(telemetry);
        telemetry.addLine("\n--------------------------------------\n");
        handler.printTo(telemetry);
        telemetry.addLine("\n--------------------------------------\n");
        flywheel.printTo(telemetry);
        telemetry.addLine("\n--------------------------------------\n");
        turret.printTo(telemetry);
    }
}
