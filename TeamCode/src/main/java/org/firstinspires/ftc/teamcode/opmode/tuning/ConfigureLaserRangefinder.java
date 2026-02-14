package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.LaserRangefinder;

@TeleOp(group = "Testing/tuning")
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
        lrf.setDistanceMode(LaserRangefinder.DistanceMode.LONG);
        int i = 6;
        lrf.setROI(0+i, 15-i, 15-i, 0+i);
        lrf.setTiming(50, 0);
        lrf.setPin0Analog(0, 4000);
        lrf.setPin1Analog(0, 4000);
    }
}

