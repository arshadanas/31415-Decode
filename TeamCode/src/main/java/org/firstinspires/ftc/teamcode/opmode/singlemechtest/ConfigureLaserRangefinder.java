package org.firstinspires.ftc.teamcode.opmode.singlemechtest;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.LaserRangefinder;

@TeleOp(group = "Brushland")
public class ConfigureLaserRangefinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LaserRangefinder lrf = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "Laser"));
        telemetry.addData("Pin0", lrf.getPin0Mode());
        telemetry.addData("Pin1", lrf.getPin1Mode());
        telemetry.addData("Distance Mode", lrf.getDistanceMode().name());
        telemetry.addData("Timing [Budget, Period]", java.util.Arrays.toString(lrf.getTiming()));
        telemetry.addData("ROI", java.util.Arrays.toString(lrf.getROI()));
        telemetry.addData("Optical Center", java.util.Arrays.toString(lrf.getOpticalCenter()));
        telemetry.update();
        waitForStart();
        /* <configuration code> */
    }
}

