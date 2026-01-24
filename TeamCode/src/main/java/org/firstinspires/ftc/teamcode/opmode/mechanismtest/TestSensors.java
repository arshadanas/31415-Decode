package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.pedropathing.Constants.localizerConstants;
import static java.lang.Math.PI;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.ArtifactColorSensor;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;

@TeleOp(group = "Single mechanism test")
public final class TestSensors extends LinearOpMode {

    @Override
    public void runOpMode() {

        mTelemetry = new MultipleTelemetry(telemetry);
        BulkReader bulkReader = new BulkReader(hardwareMap);

        Motor.Encoder
                liftEncoder = new CachedMotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_435).encoder,
                shooterEncoder = new CachedMotorEx(hardwareMap, "FR", Motor.GoBILDA.BARE).encoder,
                turretQuadrature = new CachedMotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_1150).encoder;

        liftEncoder.reset();
        shooterEncoder.reset();
        turretQuadrature.reset();

        AnalogSensor
                rotorEncoder = new AnalogSensor(hardwareMap, "rotor", 3 * 2 * PI),
                turretAbsolute = new AnalogSensor(hardwareMap, "elc", 2 * PI),
                frontDistance1 = new AnalogSensor(hardwareMap, "front 1", 1300),
                backDistance1 = new AnalogSensor(hardwareMap, "back 1", 1300);

        ArtifactColorSensor color1 = new ArtifactColorSensor(hardwareMap, "color 1a", "color 1b");
        ArtifactColorSensor color2 = new ArtifactColorSensor(hardwareMap, "color 2a", "color 2b");

        PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, localizerConstants);
        pinpoint.resetIMU();

        waitForStart();

        while (opModeIsActive()) {

            bulkReader.bulkRead();
            pinpoint.update();

            double shooterRevPerSec = shooterEncoder.getCorrectedVelocity() / 28.0;

            double turretRadQuad = normalizeRadians((turretQuadrature.getPosition() / (4 * 145.090909091)) * 2 * PI);
            double turretRadPerSec = turretQuadrature.getCorrectedVelocity() / (4 * 145.090909091) * 2 * PI;

            double rotorRad = normalizeRadians(rotorEncoder.getReading());

            double turretRadAbs = normalizeRadians(turretAbsolute.getReading());

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
