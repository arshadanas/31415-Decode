package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Artifact;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorRangefinder;

@Config
@TeleOp(group = "Testing/tuning")
public class ConfigureColorRangefinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder[] crfs = {
                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "color 1")),
//                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "color 2")),
        };
        waitForStart();
        /* Using this example configuration, you can detect both artifact colors based on which pin is reading true:
            pin0 --> purple
            pin1 --> green */
//        double scale = 255 / 360.0;
        for (int x = 0; x < crfs.length; x++) {
            crfs[x].setPin0Digital(ColorRangefinder.DigitalMode.HSV,
                    Artifact.minGreen.hue / 360.0 * 255,
                    Artifact.maxGreen.hue / 360.0 * 255
            ); // green
            crfs[x].setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 85); // 20mm or closer requirement
            Thread.sleep(3000);

            crfs[x].setPin1Digital(ColorRangefinder.DigitalMode.HSV,
                    Artifact.minPurple.hue / 360.0 * 255,
                    Artifact.maxPurple.hue / 360.0 * 255
            ); // purple
            crfs[x].setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 85); // 20mm or closer
            Thread.sleep(3000);
        }
    }
}

