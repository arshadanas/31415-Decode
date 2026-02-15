package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.LaunchZone.NONE;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;
import org.firstinspires.ftc.teamcode.subsystem.utility.Profiler;

@Config
public final class Robot {

    public final MecanumDrivetrain drivetrain;
    public final Handler handler;
    public final Shooter shooter;
    public final Turret turret;
    public final Lift lift;

    private final BulkReader bulkReader;
    private final AutoAim autoAim;
    private final ElapsedTime loopTimer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = new MecanumDrivetrain(hardwareMap, startPose);
        shooter = new Shooter(hardwareMap);
        handler = new Handler(hardwareMap, shooter.boostRPM());
        turret = new Turret(hardwareMap);
        lift = new Lift(hardwareMap);

        bulkReader = new BulkReader(hardwareMap);
        autoAim = new AutoAim();
    }

    public boolean hasArtifacts() {
        return 3 - Artifact.EMPTY.numOccurrencesIn(handler.container.artifacts) > 0;
    }

    public void setAlliance(boolean isRedAlliance) {
        autoAim.setAlliance(isRedAlliance);
    }

    public void run(boolean feed, boolean forceFeed) {
        Profiler.start("Robot_bulkread");
        bulkReader.bulkRead();
        Profiler.end("Robot_bulkread");

        boolean lifting = lift.gearSwitch.isActivated();

        if (lifting)
            turret.setTarget(PI);
        else {
            Profiler.start("dt");
            drivetrain.update();
            Profiler.end("dt");

            Profiler.start("Auto aim calc");
            autoAim.update(
                    drivetrain.getPose(),
                    drivetrain.getVelocity(),
                    drivetrain.getAngularVel(),
                    shooter.getCurrentRPM()
            );
            Profiler.end("Auto aim calc");

            turret.setTarget(autoAim.turretAngle);
            shooter.setRPM(autoAim.launchRPM);
            shooter.setLaunchAngle(autoAim.launchAngle);
        }

        boolean inLaunchZone = autoAim.currentZone != NONE;
        boolean feedsPending = handler.feedsPending();

        Profiler.start("shooter");
        shooter.run(inLaunchZone, feedsPending && !lifting);
        Profiler.end("shooter");

        Profiler.start("turret");
        turret.run(feedsPending || lifting);
        Profiler.end("turret");

        Profiler.start("handler");
        handler.run(!lifting && inLaunchZone && (
                forceFeed || (
                        feed &&
                        shooter.inTolerance(Shooter.TOLERANCE_RPM_FEEDING) &&
                        turret.inTolerance(Turret.TOLERANCE_FEEDING)
                )
        ));
        Profiler.end("handler");

        Profiler.start("lift");
        lift.run();
        Profiler.end("lift");
    }

    public void printTo(Telemetry telemetry) {
        telemetry.addData("LOOP TIME (ms)", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.addLine("\n--------------------------------------\n");
        autoAim.printTo(telemetry);
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
