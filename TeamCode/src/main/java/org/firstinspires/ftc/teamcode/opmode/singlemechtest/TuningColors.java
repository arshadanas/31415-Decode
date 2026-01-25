package org.firstinspires.ftc.teamcode.opmode.singlemechtest;

import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.PURPLE;

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
@TeleOp(group = "Single mechanism test")
public final class TuningColors extends LinearOpMode {

    public static HSV
            minPurple = new HSV(
                    175,
                    0.4,
                    0
            ),
            maxPurple = new HSV(
                    350,
                    1,
                    1
            ),

            minGreen = new HSV(
                    60,
                    0.65,
                    0
            ),
            maxGreen = new HSV(
                    160,
                    1,
                    1
            );

    public static Artifact hsvToArtifact(HSV hsv) {
        return
                hsv.between(minPurple, maxPurple) ? PURPLE :
                hsv.between(minGreen, maxGreen) ?   GREEN :
                                                    EMPTY;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

            Artifact a1 = hsvToArtifact(hsv1);
            Artifact a2 = hsvToArtifact(hsv2);
            
            mTelemetry.addData("OUTPUT", a1.or(a2));
            mTelemetry.addLine();
            mTelemetry.addData("Color 1", a1);
            mTelemetry.addData("Distance 1", distance1.getDistance(DistanceUnit.MM));
            hsv1.toTelemetry();
            mTelemetry.addLine();
            mTelemetry.addData("Color 2", a2);
            mTelemetry.addData("Distance 2", distance2.getDistance(DistanceUnit.MM));
            hsv2.toTelemetry();
            mTelemetry.update();
        }
    }
}
