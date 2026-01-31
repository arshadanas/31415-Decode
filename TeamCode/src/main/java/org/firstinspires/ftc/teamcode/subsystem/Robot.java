package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.LaunchZone.NONE;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;

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

    }

    public void run() {
        bulkReader.bulkRead();
        if (!lift.gearSwitch.isActivated())
            drivetrain.update();

        currentZone = LaunchZone.getCurrentZone(drivetrain.getPose());

        handler.run(
                currentZone != NONE,
                shooter.inTolerance(Shooter.TOLERANCE_RPM_FEEDING) &&
                turret.inTolerance(Turret.TOLERANCE_FEEDING)
        );
        shooter.run(currentZone != NONE, handler.feedsPending());
        turret.run(handler.feedsPending());

        lift.run();
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
        lift.printTo(telemetry);
    }
}
