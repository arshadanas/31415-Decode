package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.control.Ranges.clip;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public final class TuneShooter extends LinearOpMode {

    public static double targetRPM = 3000;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Shooter shooter = new Shooter(hardwareMap);

        waitForStart(); // -------------------------------------------------------------------------------------------------------------------

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            shooter.run(gamepad1.square, gamepad1.square);

            if (gamepad1.dpadUpWasPressed())
                targetRPM += 50;
            if (gamepad1.dpadDownWasPressed())
                targetRPM -= 50;

            targetRPM = clip(targetRPM, Shooter.RPM_IDLE, Shooter.RPM_MAX);

            shooter.setRPM(targetRPM);
            shooter.setManual(gamepad1.right_trigger);

            shooter.printTo(telemetry);
            telemetry.update();
        }
    }
}
