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

    public final BulkReader bulkReader;
    public final MecanumDrivetrain drivetrain;
    public final Handler handler;
    public final Shooter shooter;
    public final Turret turret;
    public final Lift lift;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private LaunchZone currentZone;

    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = new MecanumDrivetrain(hardwareMap, startPose);
        handler = new Handler(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        lift = new Lift(hardwareMap);

        bulkReader = new BulkReader(hardwareMap);
    }

    public void setAlliance(boolean isRedAlliance) {
        AutoAim.isRedAlliance = isRedAlliance;
    }

    public void run(boolean feed) {
        Profiler.start("Robot_bulkread");
        bulkReader.bulkRead();
        Profiler.end("Robot_bulkread");

        if (lift.gearSwitch.isActivated()) {
            currentZone = NONE;
            turret.setTarget(PI);
        } else {
            Profiler.start("dt");
            drivetrain.update();
            Profiler.end("dt");

            Profiler.start("Auto aim calc");
            AutoAim.update(
                    drivetrain.getPose(),
                    drivetrain.getVelocity(),
                    drivetrain.getAngularVel(),
                    shooter.getCurrentRPM()
            );
            Profiler.end("Auto aim calc");

            currentZone = AutoAim.currentZone;
            turret.setTarget(AutoAim.turretAngle);
            shooter.setRPM(AutoAim.launchRPM);
            shooter.setLaunchAngle(AutoAim.launchAngle);
        }

        boolean inLaunchZone = true; //currentZone != NONE; TODO enable

        Profiler.start("shooter");
        shooter.run(inLaunchZone, handler.feedsPending());
        Profiler.end("shooter");

        Profiler.start("turret");
        turret.run(handler.feedsPending());
        Profiler.end("turret");

        Profiler.start("handler");
        handler.run(// TODO enable rpm + direction tolerance checking
                inLaunchZone,
                feed //&&
//                shooter.inTolerance(Shooter.TOLERANCE_RPM_FEEDING) //&&
//                turret.inTolerance(Turret.TOLERANCE_FEEDING)
        );
        Profiler.end("handler");

        Profiler.start("lift");
        lift.run();
        Profiler.end("lift");
    }

    public void printTo(Telemetry telemetry) {
        telemetry.addData("LOOP TIME (ms)", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.addLine();
        telemetry.addData("CURRENT ZONE", currentZone);
        telemetry.addData("Shooter-goal dist (in)", AutoAim.rMag);
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
