package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.control.Ranges.clip;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Tele;

@Config
@TeleOp(group = "Testing/tuning")
public final class TuneShooter extends LinearOpMode {

    private final ElapsedTime loopTimer = new ElapsedTime();
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
            shooter.run(!gamepad1.square, !gamepad1.square);

            if (gamepad1.dpadUpWasPressed())
                targetRPM += 50;
            if (gamepad1.dpadDownWasPressed())
                targetRPM -= 50;

            targetRPM = clip(targetRPM, 0, 8000);

            shooter.setRPM(targetRPM);
            shooter.setManual(gamepad1.right_trigger);

            Thread.sleep((long)(max(Tele.AVG_LOOP_TIME_MS - loopTimer.milliseconds(),0)));
            telemetry.addData("LOOP TIME", loopTimer.seconds());
            loopTimer.reset();
            shooter.printTo(telemetry);
            telemetry.update();
        }
    }
}
