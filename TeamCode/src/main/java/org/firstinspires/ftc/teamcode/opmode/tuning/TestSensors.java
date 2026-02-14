package org.firstinspires.ftc.teamcode.opmode.tuning;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.pedropathing.Constants.pinpointConstants;
import static org.firstinspires.ftc.teamcode.subsystem.Container.ROTOR_ENCODER_OFFSET;
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
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.DigitalArtifactColor;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;

@Config
@TeleOp(group = "Testing/tuning")
public final class TestSensors extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());;
        
        BulkReader bulkReader = new BulkReader(hardwareMap);

        Motor.Encoder
                liftEncoder = new CachedMotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_435).encoder,
                shooterEncoder = new CachedMotorEx(hardwareMap, "BR", Motor.GoBILDA.BARE).encoder,
                turretQuadrature = new CachedMotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_1150).encoder;
        liftEncoder.setDirection(REVERSE);
        turretQuadrature.setDirection(REVERSE);

        liftEncoder.reset();
//        shooterEncoder.reset(); this breaks velo measurements
        turretQuadrature.reset();

        AnalogSensor
                rotorEncoder = new AnalogSensor(hardwareMap, "rotor", 2 * PI),
                turretAbsolute = new AnalogSensor(hardwareMap, "elc", 2 * PI),
                frontDistance1 = new AnalogSensor(hardwareMap, "front 1", 4000),
                backDistance1 = new AnalogSensor(hardwareMap, "back 1", 4000);

        DigitalArtifactColor color1 = new DigitalArtifactColor(hardwareMap, "color 1a", "color 1b"); // TODO CONFIGURE
        DigitalArtifactColor color2 = new DigitalArtifactColor(hardwareMap, "color 2a", "color 2b"); // TODO CONFIGURE

        DigitalChannel color1a = hardwareMap.digitalChannel.get("color 1a");
        DigitalChannel color1b = hardwareMap.digitalChannel.get("color 1b");
        DigitalChannel color2a = hardwareMap.digitalChannel.get("color 2a");
        DigitalChannel color2b = hardwareMap.digitalChannel.get("color 2b");

        PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, pinpointConstants);
        pinpoint.resetIMU();

        waitForStart();

        while (opModeIsActive()) {

            bulkReader.bulkRead();
            pinpoint.update();

            double shooterRevPerSec = shooterEncoder.getCorrectedVelocity() / 28.0;

            double turretRadPerTick = 2 * PI / (4 * 145.090909091);
            double turretRadQuad = normalizeRadians(turretQuadrature.getPosition() * turretRadPerTick);
            double turretRadPerSec = turretQuadrature.getCorrectedVelocity() * turretRadPerTick;

            double rotorRad = normalizeRadians(rotorEncoder.getReading() + ROTOR_ENCODER_OFFSET);

            double turretRadAbs = normalizeRadians(-turretAbsolute.getReading() + Turret.TURRET_ABSOLUTE_OFFSET);

            Pose pose = pinpoint.getPose();

            telemetry.addLine("QUADRATURE ENCODERS");
            telemetry.addLine();
            telemetry.addData("Lift position (ticks)", liftEncoder.getPosition());
            telemetry.addLine();
            telemetry.addData("Shooter velocity (rpm)", shooterRevPerSec * 60);
            telemetry.addData("Shooter velocity (rad/s)", shooterRevPerSec * 2 * PI);
            telemetry.addLine();
            telemetry.addData("Turret quad position (rad)", turretRadQuad);
            telemetry.addData("Turret quad position (deg)", toDegrees(turretRadQuad));
            telemetry.addData("Turret velocity (rad/s)", turretRadPerSec);
            telemetry.addData("Turret velocity (deg/s)", toDegrees(turretRadPerSec));
            telemetry.addLine();
            telemetry.addLine("--------------------------------------");
            telemetry.addLine();
            telemetry.addLine("ANALOG SENSORS");
            telemetry.addLine();
            telemetry.addData("Rotor position (rad)", rotorRad);
            telemetry.addData("Rotor position (deg)", toDegrees(rotorRad));
            telemetry.addLine();
            telemetry.addData("Turret abs position (rad)", turretRadAbs);
            telemetry.addData("Turret abs position (deg)", toDegrees(turretRadAbs));
            telemetry.addLine();
            telemetry.addData("Front distance (mm)", frontDistance1.getReading());
            telemetry.addData("Back distance (mm)", backDistance1.getReading());
            telemetry.addLine();
            telemetry.addLine("--------------------------------------");
            telemetry.addLine();
            telemetry.addLine("DIGITAL SENSORS");
            telemetry.addLine();
            telemetry.addData("Color 1", color1.getArtifact());
            telemetry.addData("Color 1a", color1a.getState());
            telemetry.addData("Color 1b", color1b.getState());
            telemetry.addLine();
            telemetry.addData("Color 2", color2.getArtifact());
            telemetry.addData("Color 2a", color2a.getState());
            telemetry.addData("Color 2b", color2b.getState());
            telemetry.addLine();
            telemetry.addLine("--------------------------------------");
            telemetry.addLine();
            telemetry.addLine("DRIVETRAIN:");
            telemetry.addLine();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading (rad)", pose.getHeading());
            telemetry.addData("Heading (deg)", toDegrees(pose.getHeading()));

            telemetry.update();
        }
    }
}
