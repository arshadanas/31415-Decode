package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmode.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;

@TeleOp(group = "Testing/tuning")
public final class TestFTCLibMecanum extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:
        BulkReader bulkReader = new BulkReader(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // Read stuff
            bulkReader.bulkRead();

            drivetrain.run(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

            telemetry.addData("LOOP TIME", timer.milliseconds()); timer.reset();

            telemetry.addData("Heading angle (DEGREES)", drivetrain.pinpoint.getHeading(AngleUnit.DEGREES));

            telemetry.update();
        }
    }
}
