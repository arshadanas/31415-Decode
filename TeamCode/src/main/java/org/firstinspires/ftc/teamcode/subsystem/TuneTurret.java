package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public final class TuneTurret extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(hardwareMap);

        waitForStart(); // -------------------------------------------------------------------------------------------------------------------

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            turret.run(!gamepad1.square);

            float x = gamepad1.right_stick_x, y = gamepad1.right_stick_y;
            if (x*x + y*y >= 0.64)
                turret.setTarget(PI + atan2(x, y));

            if (gamepad1.dpadUpWasPressed())
                turret.setTarget(0);
            else if (gamepad1.dpadLeftWasPressed())
                turret.setTarget(PI/2);
            else if (gamepad1.dpadDownWasPressed())
                turret.setTarget(PI);
            else if (gamepad1.dpadRightWasPressed())
                turret.setTarget(-PI/2);

            turret.printTo(telemetry);
            telemetry.update();
        }
    }
}
