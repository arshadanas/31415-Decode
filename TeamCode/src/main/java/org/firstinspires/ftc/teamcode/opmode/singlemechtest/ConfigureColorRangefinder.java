package org.firstinspires.ftc.teamcode.opmode.singlemechtest;

import static org.firstinspires.ftc.teamcode.opmode.singlemechtest.TuningColors.maxGreen;
import static org.firstinspires.ftc.teamcode.opmode.singlemechtest.TuningColors.maxPurple;
import static org.firstinspires.ftc.teamcode.opmode.singlemechtest.TuningColors.minGreen;
import static org.firstinspires.ftc.teamcode.opmode.singlemechtest.TuningColors.minPurple;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorRangefinder;

@Config
@TeleOp(group = "Brushland")
public class ConfigureColorRangefinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder[] crfs = {
                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "color 1")),
                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "color 2")),
        };
        waitForStart();
        /* Using this example configuration, you can detect both artifact colors based on which pin is reading true:
            pin0 --> purple
            pin1 --> green */
        double scale = 255 / 360.0;
        for (ColorRangefinder crf : crfs) {
            crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, minPurple.hue * scale, maxPurple.hue * scale); // purple
            crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 20); // 20mm or closer

            crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, minGreen.hue * scale, maxGreen.hue * scale); // green
            crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 20); // 20mm or closer requirement
        }
    }
}

