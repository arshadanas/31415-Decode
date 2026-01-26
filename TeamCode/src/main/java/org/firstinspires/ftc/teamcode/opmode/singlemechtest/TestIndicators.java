package org.firstinspires.ftc.teamcode.opmode.singlemechtest;

import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Artifact;
import org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator;


@TeleOp(group = "Single mechanism test")
public final class TestIndicators extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:

        LEDIndicator[] indicators = {
                new LEDIndicator(hardwareMap, "led 1a", "led 1b"),
                new LEDIndicator(hardwareMap, "led 2a", "led 2b"),
                new LEDIndicator(hardwareMap, "led 3a", "led 3b")
        };

        Artifact[] slots = {EMPTY, EMPTY, EMPTY};
        Artifact[] filtered = new Artifact[slots.length];

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpadLeftWasPressed())
                slots[0] = slots[0].plus(1);
            if (gamepad1.dpadUpWasPressed())
                slots[1] = slots[1].plus(1);
            if (gamepad1.dpadRightWasPressed())
                slots[2] = slots[2].plus(1);


            int n = 0;
            for (Artifact a : slots) {
                if (a != EMPTY)
                    filtered[n++] = a;
            }
            while (n < filtered.length)
                filtered[n++] = EMPTY;

            for (int i = 0; i < indicators.length; i++)
                indicators[i].setColor(filtered[i].toLEDColor());

            telemetry.addData("Slot 0", slots[0]);
            telemetry.addData("Slot 1", slots[1]);
            telemetry.addData("Slot 2", slots[2]);
            telemetry.update();
        }
    }
}
