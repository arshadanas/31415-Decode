package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.Container.Position.FEEDING;
import static org.firstinspires.ftc.teamcode.subsystem.Container.Position.INTAKING;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public final class TuneContainer extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Container container = new Container(hardwareMap);

        waitForStart();

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            container.run();

            if (gamepad1.dpadLeftWasPressed())
                container.moveSlot(0, FEEDING);

            else if (gamepad1.dpadUpWasPressed())
                container.moveSlot(1, FEEDING);

            else if (gamepad1.dpadRightWasPressed())
                container.moveSlot(2, FEEDING);

            else if (gamepad1.squareWasPressed())
                container.moveSlot(0, INTAKING);

            else if (gamepad1.triangleWasPressed())
                container.moveSlot(1, INTAKING);

            else if (gamepad1.circleWasPressed())
                container.moveSlot(2, INTAKING);

            container.print(telemetry);
            telemetry.update();
        }
    }
}
