package org.firstinspires.ftc.teamcode.opmode.singlemechtest;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.control.motion.PIDDriver;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;


//@TeleOp(group = "Single mechanism test")
//@Config
public final class TuningPIDDriver extends LinearOpMode {

    public static EditablePose target = new EditablePose(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:

        BulkReader bulkReader = new BulkReader(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        Drivetrain drivetrain = new MecanumDrivetrain(hardwareMap);

        PIDDriver driver = new PIDDriver();

        waitForStart();

        while (opModeIsActive()) {
            bulkReader.bulkRead();

//            Pose2d current = drivetrain.getPoseEstimate();

            if (gamepad1.x) {
//                boolean done = driver.driveTo(drivetrain, target.toPose2d());

//                telemetry.addLine(done ? "Target reached" : "Moving to target");
                telemetry.addLine();
            }
//            telemetry.addData("Current X", current.getX());
//            telemetry.addData("Current Y", current.getY());
//            telemetry.addData("Current heading", normalizeRadians(current.getHeading()));
            telemetry.addLine();
            telemetry.addData("Target X", target.x);
            telemetry.addData("Target Y", target.y);
            telemetry.addData("Target heading", normalizeRadians(target.heading));
            telemetry.update();
        }
    }
}
