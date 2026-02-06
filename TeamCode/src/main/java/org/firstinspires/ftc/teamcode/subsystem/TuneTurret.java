package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.max;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "Testing/tuning")
public final class TuneTurret extends LinearOpMode {

    private final ElapsedTime loopTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(hardwareMap);
        turret.motor.encoder.setDirection(Motor.Direction.REVERSE);

        waitForStart(); // -------------------------------------------------------------------------------------------------------------------

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            turret.run(gamepad1.square);

            float x = gamepad1.right_stick_x, y = gamepad1.right_stick_y;
            if (x*x + y*y >= 0.64)
                turret.setTarget(PI + atan2(x, y));

            if (gamepad1.dpadUpWasPressed())
                turret.setTarget(0);
            else if (gamepad1.dpadLeftWasPressed())
                turret.setTarget(toRadians(160));
//            else if (gamepad1.dpadDownWasPressed())
//                turret.setTarget(toRadians());
            else if (gamepad1.dpadRightWasPressed())
                turret.setTarget(toRadians(-160));

            Thread.sleep((long)(max(35 - loopTimer.milliseconds(),0)));
            telemetry.addData("LOOP TIME", loopTimer.seconds());
            loopTimer.reset();
            turret.printTo(telemetry);
            telemetry.update();
        }
    }
}
