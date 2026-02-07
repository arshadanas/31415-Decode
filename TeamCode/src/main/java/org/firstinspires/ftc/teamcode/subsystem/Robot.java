package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.SIZE_FIELD;
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

import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;
import org.firstinspires.ftc.teamcode.subsystem.utility.Profiler;

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

    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = new MecanumDrivetrain(hardwareMap, startPose);
        handler = new Handler(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        lift = new Lift(hardwareMap);

        bulkReader = new BulkReader(hardwareMap);
    }

    boolean isRedAlliance = false;

    public void setAlliance(boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;
    }

    public void run(boolean feed) {
        Profiler.start("Robot_bulkread");
        bulkReader.bulkRead();
        Profiler.end("Robot_bulkread");

        if (!lift.gearSwitch.isActivated()) {
            Profiler.start("dt");
            drivetrain.update();
            Profiler.end("dt");

            Profiler.start("aim_turret");

            Pose currentPose = drivetrain.getPose();
            Vector2 poseVec = new Vector2(currentPose.getX(), currentPose.getY());
            Vector2 goalVec = isRedAlliance ?
                    new Vector2(SIZE_FIELD - 10, SIZE_FIELD) // Red side
                    : new Vector2(10, SIZE_FIELD); // Blue side
            Vector2 aimVec = goalVec.difference(poseVec);

            Vector2 robotOrientation = new Vector2(currentPose.getHeading());

            turret.setTarget(robotOrientation.getAngleBetween(aimVec));

            Profiler.end("aim_turret");
        }
        
        Profiler.start("GetCurrentZone");
        currentZone = LaunchZone.getCurrentZone(drivetrain.getPose());
        Profiler.end("GetCurrentZone");

        Profiler.start("setShooterRPMAngle");
        switch (currentZone) {
//        switch (NEAR) {
            case NEAR:
                shooter.setRPM(RPM_NEAR);
                shooter.setLaunchAngle(LAUNCH_RAD_NEAR);
                break;
            case FAR:
                shooter.setRPM(RPM_FAR);
                shooter.setLaunchAngle(LAUNCH_RAD_FAR);
                break;
        }
        Profiler.end("setShooterRPMAngle");

        Profiler.start("shooter");
        boolean inLaunchZone = true, a = currentZone != NONE;
        shooter.run(inLaunchZone, handler.feedsPending());
        Profiler.end("shooter");

        Profiler.start("turret");
        turret.run(handler.feedsPending());
        Profiler.end("turret");

        Profiler.start("handler");
        handler.run(
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
