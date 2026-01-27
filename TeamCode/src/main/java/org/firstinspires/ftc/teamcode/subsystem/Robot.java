package org.firstinspires.ftc.teamcode.subsystem;

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
    public final Lift lift;

    private final ElapsedTime loopTimer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap, Pose startPose) {
        drivetrain = new MecanumDrivetrain(hardwareMap, startPose);
        lift = new Lift(hardwareMap);

        bulkReader = new BulkReader(hardwareMap);
    }

    public void setAlliance(boolean isRedAlliance) {

    }

    public void run() {
        bulkReader.bulkRead();
        if (!lift.enabled) drivetrain.update();
        lift.run();
    }

    public void print(Telemetry telemetry) {
        telemetry.addData("LOOP TIME", loopTimer.seconds());
        loopTimer.reset();
        telemetry.addLine();
        drivetrain.print(telemetry);
    }
}
