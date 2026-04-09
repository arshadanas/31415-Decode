package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.LaunchZone.NONE;

import static java.lang.Math.PI;

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
    public final Shooter shooter;
    public final Turret turret;
    public final Lift lift;

    private final BulkReader bulkReader;
    private final AirtimeSolver solver;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private LaunchZone currentZone;

    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = new MecanumDrivetrain(hardwareMap, startPose);
        shooter = new Shooter(hardwareMap);
        handler = new Handler(hardwareMap);
        turret = new Turret(hardwareMap);
        lift = new Lift(hardwareMap);

        bulkReader = new BulkReader(hardwareMap);
        solver = new AirtimeSolver();
    }

    public void setAlliance(boolean isRedAlliance) {
        solver.setAlliance(isRedAlliance);
    }

    public void run(boolean feed, boolean forceFeed) {
        bulkReader.bulkRead();

        boolean lifting = lift.gearSwitch.isActivated();

        if (lifting)
            turret.setTarget(PI);
        else {
            drivetrain.update();
            Pose pose = drivetrain.getPose();
            Vector velocity = drivetrain.getVelocity();

            currentZone = LaunchZone.getCurrentZone(pose);

            solver.update(
                    pose.getX(), pose.getY(), pose.getHeading(),
                    velocity.getXComponent(), velocity.getYComponent(), drivetrain.getAngularVel()
            );

            turret.setTarget(solver.turretAngle);
            shooter.setRPM(solver.launchRPM);
            shooter.setLaunchAngle(solver.launchAngle);
        }

        boolean inLaunchZone = currentZone != NONE;
        boolean feedsPending = handler.feedsPending();

        shooter.run(inLaunchZone, feedsPending && !lifting);
        turret.run(feedsPending || lifting);

        boolean inTolerance = shooter.inTolerance(Shooter.TOLERANCE_RPM_FEEDING) &&
                                turret.inTolerance(Turret.TOLERANCE_FEEDING);

        handler.run(!lifting && inLaunchZone && (forceFeed || (feed && inTolerance)));
        lift.run();
    }

    public void printTo(Telemetry telemetry) {
        telemetry.addData("LOOP TIME (ms)", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.addLine("\n--------------------------------------\n");
        telemetry.addData("Current zone", currentZone);
        telemetry.addLine();
        solver.printTo(telemetry);
        telemetry.addLine("\n--------------------------------------\n");
        drivetrain.printTo(telemetry);
        telemetry.addLine("\n--------------------------------------\n");
        handler.printTo(telemetry);
        telemetry.addLine("\n--------------------------------------\n");
        shooter.printTo(telemetry);
        telemetry.addLine("\n--------------------------------------\n");
        turret.printTo(telemetry);
        telemetry.addLine("\n--------------------------------------\n");
        lift.printTo(telemetry);
    }
}
