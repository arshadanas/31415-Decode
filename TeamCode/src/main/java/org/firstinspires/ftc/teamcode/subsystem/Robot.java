package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.LaunchZone.NEAR;
import static org.firstinspires.ftc.teamcode.subsystem.LaunchZone.NONE;
import static org.firstinspires.ftc.teamcode.subsystem.Shooter.LAUNCH_RAD_FAR;
import static org.firstinspires.ftc.teamcode.subsystem.Shooter.LAUNCH_RAD_NEAR;
import static org.firstinspires.ftc.teamcode.subsystem.Shooter.RPM_FAR;
import static org.firstinspires.ftc.teamcode.subsystem.Shooter.RPM_NEAR;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;
import org.firstinspires.ftc.teamcode.subsystem.utility.ToggleProfiler;

import dev.nullftc.profiler.Profiler;

@Config
public final class Robot {
// TODO gate pressers
    public final BulkReader bulkReader;
    public final MecanumDrivetrain drivetrain;
    public final Handler handler;
    public final Shooter shooter;
    public final Turret turret;
    public final Lift lift;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private LaunchZone currentZone;

    private final ToggleProfiler profiler;

    public Robot(HardwareMap hardwareMap, Pose startPose, Profiler profiler) {
        this.profiler = new ToggleProfiler(profiler);

        drivetrain = new MecanumDrivetrain(hardwareMap, startPose);
        handler = new Handler(hardwareMap);
        shooter = new Shooter(hardwareMap, this.profiler);
        turret = new Turret(hardwareMap);
        lift = new Lift(hardwareMap);

        bulkReader = new BulkReader(hardwareMap);
    }

    public void setAlliance(boolean isRedAlliance) {

    }

    public void run(boolean feed) {
        profiler.start("bulkread");
        bulkReader.bulkRead();
        profiler.end("bulkread");

        if (!lift.gearSwitch.isActivated())
        {
            profiler.start("dt");
            drivetrain.update();
            profiler.end("dt");
        }
        profiler.start("getcurrentzone");
        currentZone = NEAR;
                LaunchZone.getCurrentZone(drivetrain.getPose());

        currentZone = LaunchZone.getCurrentZone(drivetrain.getPose());

        profiler.end("getcurrentzone");

        profiler.start("setShooterRPMAngle");
        switch (currentZone) {
        switch (NEAR) {
            case NEAR:
                shooter.setRPM(RPM_NEAR);
                shooter.setLaunchAngle(LAUNCH_RAD_NEAR);
                break;
            case FAR:
                shooter.setRPM(RPM_FAR);
                shooter.setLaunchAngle(LAUNCH_RAD_FAR);
                break;
        }
        profiler.end("setShooterRPMAngle");

        profiler.start("shooter");
        shooter.run(currentZone != NONE, handler.feedsPending());
        profiler.end("shooter");

        profiler.start("turret");
        boolean inLaunchZone = true, a = currentZone != NONE;
        shooter.run(inLaunchZone, handler.feedsPending());
        turret.run(handler.feedsPending());
        profiler.end("turret");

        profiler.start("handler");
        handler.run(
                inLaunchZone,
                feed //&&
//                shooter.inTolerance(Shooter.TOLERANCE_RPM_FEEDING) //&&
//                turret.inTolerance(Turret.TOLERANCE_FEEDING)
        );
        profiler.end("handler");

        profiler.start("lift");
        lift.run();
        profiler.end("lift");
    }

    public void printTo(Telemetry telemetry) {
        telemetry.addData("LOOP TIME", loopTimer.seconds());
        loopTimer.reset();
        telemetry.addLine();
        telemetry.addData("CURRENT ZONE", currentZone);
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
