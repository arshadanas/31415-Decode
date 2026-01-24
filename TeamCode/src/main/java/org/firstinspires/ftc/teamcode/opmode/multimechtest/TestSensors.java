package org.firstinspires.ftc.teamcode.opmode.multimechtest;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.pedropathing.Constants.localizerConstants;
import static java.lang.Math.PI;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.ArtifactColorSensor;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;

import java.util.ArrayList;

@Config
@TeleOp(group = "Multiple mechanism test")
public final class TestSensors extends LinearOpMode {

    public static double turretAbsoluteOffset = 1.8659156366775742;

    @Override
    public void runOpMode() {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());;
        
        BulkReader bulkReader = new BulkReader(hardwareMap);

        Motor.Encoder
                liftEncoder = new CachedMotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_435).encoder,
                shooterEncoder = new CachedMotorEx(hardwareMap, "FR", Motor.GoBILDA.BARE).encoder,
                turretQuadrature = new CachedMotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_1150).encoder;
        liftEncoder.setDirection(REVERSE);
        turretQuadrature.setDirection(REVERSE);

        liftEncoder.reset();
        shooterEncoder.reset();
        turretQuadrature.reset();

        AnalogSensor
                rotorEncoder = new AnalogSensor(hardwareMap, "rotor", 3 * 2 * PI),
                turretAbsolute = new AnalogSensor(hardwareMap, "elc", 2 * PI), // TODO FIX
                frontDistance1 = new AnalogSensor(hardwareMap, "front 1", 1300), // TODO CONFIGURE
                backDistance1 = new AnalogSensor(hardwareMap, "back 1", 1000);

        ArtifactColorSensor color1 = new ArtifactColorSensor(hardwareMap, "color 1a", "color 1b"); // TODO CONFIGURE
        ArtifactColorSensor color2 = new ArtifactColorSensor(hardwareMap, "color 2a", "color 2b"); // TODO CONFIGURE

        PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, localizerConstants);
        pinpoint.resetIMU();

        ArrayList<Double> absReadings = new ArrayList<>();

        waitForStart();

        while (opModeIsActive()) {

            bulkReader.bulkRead();
            pinpoint.update();

            double shooterRevPerSec = shooterEncoder.getCorrectedVelocity() / 28.0;

            double turretRadPerTick = 2 * PI / (4 * 145.090909091);
            double turretRadQuad = normalizeRadians(turretQuadrature.getPosition() * turretRadPerTick);
            double turretRadPerSec = turretQuadrature.getCorrectedVelocity() * turretRadPerTick;

            double rotorRad = normalizeRadians(rotorEncoder.getReading());

            double turretRadAbsRaw = -turretAbsolute.getReading();
            double turretRadAbs = normalizeRadians(turretRadAbsRaw + turretAbsoluteOffset);

            if (gamepad1.square)
                absReadings.add(turretRadAbsRaw);
            if (gamepad1.circleWasPressed()) {
                double sum = 0;
                for (double read : absReadings)
                    sum += read;
                turretAbsoluteOffset = -sum / absReadings.size();
            }

            Pose pose = pinpoint.getPose();

            mTelemetry.addLine("QUADRATURE ENCODERS");
            mTelemetry.addLine();
            mTelemetry.addData("Lift position (ticks)", liftEncoder.getPosition());
            mTelemetry.addLine();
            mTelemetry.addData("Shooter velocity (rpm)", shooterRevPerSec * 60);
            mTelemetry.addData("Shooter velocity (rad/s)", shooterRevPerSec * 2 * PI);
            mTelemetry.addLine();
            mTelemetry.addData("Turret quad position (rad)", turretRadQuad);
            mTelemetry.addData("Turret quad position (deg)", toDegrees(turretRadQuad));
            mTelemetry.addData("Turret velocity (rad/s)", turretRadPerSec);
            mTelemetry.addData("Turret velocity (deg/s)", toDegrees(turretRadPerSec));

            divider();

            mTelemetry.addLine("ANALOG SENSORS");
            mTelemetry.addLine();
            mTelemetry.addData("Rotor position (rad)", rotorRad);
            mTelemetry.addData("Rotor position (deg)", toDegrees(rotorRad));
            mTelemetry.addLine();
            mTelemetry.addData("Turret abs position (rad)", turretRadAbs);
            mTelemetry.addData("Turret abs position (deg)", toDegrees(turretRadAbs));
            mTelemetry.addLine();
            mTelemetry.addData("Front distance (mm)", frontDistance1.getReading());
            mTelemetry.addData("Back distance (mm)", backDistance1.getReading());

            divider();

            mTelemetry.addLine("DIGITAL SENSORS");
            mTelemetry.addLine();
            mTelemetry.addData("Color 1", color1.getArtifact());
            mTelemetry.addData("Color 2", color2.getArtifact());

            divider();

            mTelemetry.addLine("DRIVETRAIN:");
            mTelemetry.addLine();
            mTelemetry.addData("X", pose.getX());
            mTelemetry.addData("Y", pose.getY());
            mTelemetry.addData("Heading (rad)", pose.getHeading());
            mTelemetry.addData("Heading (deg)", toDegrees(pose.getHeading()));


            mTelemetry.update();
        }
    }
}
