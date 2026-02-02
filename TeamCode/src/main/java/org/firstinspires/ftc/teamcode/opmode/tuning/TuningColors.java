package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.subsystem.Artifact;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

@Config
@TeleOp(group = "Testing/tuning")
public final class TuningColors extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ColorSensor color1 = new ColorSensor(hardwareMap, "color 1", 1);
        ColorSensor color2 = new ColorSensor(hardwareMap, "color 2", 1);

        DistanceSensor distance1 = hardwareMap.get(DistanceSensor.class, "color 1");
        DistanceSensor distance2 = hardwareMap.get(DistanceSensor.class, "color 2");

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {

            color1.update();
            color2.update();

            HSV hsv1 = color1.getHSV();
            HSV hsv2 = color2.getHSV();

            Artifact a1 = Artifact.fromHSV(hsv1);
            Artifact a2 = Artifact.fromHSV(hsv2);
            
            telemetry.addData("OUTPUT", a1.or(a2));
            telemetry.addLine();
            telemetry.addData("Color 1", a1);
            telemetry.addData("Distance 1", distance1.getDistance(DistanceUnit.MM));
            hsv1.print(telemetry);
            telemetry.addLine();
            telemetry.addData("Color 2", a2);
            telemetry.addData("Distance 2", distance2.getDistance(DistanceUnit.MM));
            hsv2.print(telemetry);
            telemetry.update();
        }
    }
}
