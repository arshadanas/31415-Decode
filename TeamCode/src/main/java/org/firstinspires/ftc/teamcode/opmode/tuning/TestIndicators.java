package org.firstinspires.ftc.teamcode.opmode.tuning;

import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.GREEN;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Artifact;
import org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator;


@Disabled
@TeleOp(group = "Testing/tuning")
public final class TestIndicators extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:

        LEDIndicator[] indicators = {
                new LEDIndicator(hardwareMap, "led 1a", "led 1b"),
                new LEDIndicator(hardwareMap, "led 2a", "led 2b"),
                new LEDIndicator(hardwareMap, "led 3a", "led 3b")
        };

        Artifact[] artifacts = {EMPTY, EMPTY, EMPTY};

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpadLeftWasPressed())
                artifacts[0] = artifacts[0].plus(1);
            if (gamepad1.dpadUpWasPressed())
                artifacts[1] = artifacts[1].plus(1);
            if (gamepad1.dpadRightWasPressed())
                artifacts[2] = artifacts[2].plus(1);


            int n = 0;
            for (Artifact a : artifacts) {
                if (a != EMPTY)
                    indicators[n++].setColor(a == GREEN ? LEDIndicator.LEDColor.GREEN : LEDIndicator.LEDColor.RED);
            }
            while (n < artifacts.length)
                indicators[n++].setColor(LEDIndicator.LEDColor.OFF);

            telemetry.addData("Slot 0", artifacts[0]);
            telemetry.addData("Slot 1", artifacts[1]);
            telemetry.addData("Slot 2", artifacts[2]);
            telemetry.update();
        }
    }
}
