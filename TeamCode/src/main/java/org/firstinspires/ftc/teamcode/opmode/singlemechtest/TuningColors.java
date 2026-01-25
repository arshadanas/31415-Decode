package org.firstinspires.ftc.teamcode.opmode.singlemechtest;

import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.subsystem.Artifact;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

@TeleOp(group = "Single mechanism test")
public final class TuningColors extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ColorSensor color1 = new ColorSensor(hardwareMap, "color 1", 1);
        ColorSensor color2 = new ColorSensor(hardwareMap, "color 2", 1);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {

            color1.update();
            color2.update();

            HSV hsv1 = color1.getHSV();
            HSV hsv2 = color2.getHSV();

            mTelemetry.addData("Color 1", Artifact.fromHSV(hsv1));
            hsv1.toTelemetry();
            mTelemetry.addLine();
            mTelemetry.addData("Color 2", Artifact.fromHSV(hsv2));
            hsv2.toTelemetry();
            mTelemetry.update();
        }
    }
}
