package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Math.max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Tele;

@TeleOp(group = "Testing/tuning")
public final class TuneRotor extends LinearOpMode {

    private final ElapsedTime loopTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Rotor rotor = new Rotor(hardwareMap);

        waitForStart();

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            rotor.run();

            if (gamepad1.dpadLeftWasPressed())
                rotor.moveSlot(0, Rotor.Zone.FEEDER);

            else if (gamepad1.dpadUpWasPressed())
                rotor.moveSlot(1, Rotor.Zone.FEEDER);

            else if (gamepad1.dpadRightWasPressed())
                rotor.moveSlot(2, Rotor.Zone.FEEDER);

            else if (gamepad1.squareWasPressed())
                rotor.moveSlot(0, Rotor.Zone.INTAKE_SENSOR);

            else if (gamepad1.triangleWasPressed())
                rotor.moveSlot(1, Rotor.Zone.INTAKE_SENSOR);

            else if (gamepad1.circleWasPressed())
                rotor.moveSlot(2, Rotor.Zone.INTAKE_SENSOR);

            rotor.printTo(telemetry);

            Thread.sleep((long)(max(Tele.AVG_LOOP_TIME_MS - loopTimer.milliseconds(),0)));
            telemetry.addData("LOOP TIME", loopTimer.seconds());
            loopTimer.reset();
            telemetry.update();


        }
    }
}
