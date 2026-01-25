package org.firstinspires.ftc.teamcode.opmode.singlemechtest;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorRangefinder;


@TeleOp(group = "Brushland")
public class ConfigureColorRangefinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder[] crfs = {
                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "Color 1")),
                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "Color 2")),
        };
        waitForStart();
        /* Using this example configuration, you can detect both artifact colors based on which pin is reading true:
            pin0 --> purple
            pin1 --> green */
        for (ColorRangefinder crf : crfs) {
            crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 160 / 360.0 * 255, 190 / 360.0 * 255); // purple
            crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 10); // 10mm or closer

            crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 110 / 360.0 * 255, 140 / 360.0 * 255); // green
            crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 10); // 10mm or closer requirement
        }
    }
}

