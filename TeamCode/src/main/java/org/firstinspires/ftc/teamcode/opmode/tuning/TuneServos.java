package org.firstinspires.ftc.teamcode.opmode.tuning;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.control.Ranges.wrap;
import static org.firstinspires.ftc.teamcode.opmode.tuning.TuneServos.TestServo.HOOD;
import static org.firstinspires.ftc.teamcode.opmode.tuning.TuneServos.TestServo.ROTOR;
import static org.firstinspires.ftc.teamcode.subsystem.Shooter.ANGLE_HOOD_SHALLOWEST;
import static org.firstinspires.ftc.teamcode.subsystem.Shooter.ANGLE_HOOD_STEEPEST;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Rotor;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
@TeleOp(group = "Testing/tuning")
public final class TuneServos extends LinearOpMode {

    public static double rotorAngle;

    enum TestServo {
        HOOD, ROTOR;

        private final static TestServo[] values = values();

        public TestServo plus(int n) {
            return values[wrap(ordinal() + n, 0, values.length)];
        }

        public String markIf(TestServo s) {
            return this == s ? "> " : "  ";
        }
    }

    @Override
    public void runOpMode() {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        CachedSimpleServo hood = new CachedSimpleServo(hardwareMap, "hood", 0, 360).reversed();
        CachedSimpleServo rotor = new CachedSimpleServo(hardwareMap, "rotor 2", -PI, PI);

        boolean hoodMax = false;

        TestServo selected = HOOD;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpadUpWasPressed()) selected = selected.plus(-1);
            if (gamepad1.dpadDownWasPressed()) selected = selected.plus(1);

            if (gamepad1.squareWasPressed()) switch (selected) {
                case HOOD:
                    hoodMax = !hoodMax;
                    break;

                case ROTOR:
                    rotorAngle = normalizeRadians(rotorAngle + PI / 3);
                    break;
            }

            hood.turnToAngle(hoodMax ? ANGLE_HOOD_SHALLOWEST : ANGLE_HOOD_STEEPEST);

            rotor.turnToAngle(normalizeRadians(rotorAngle + Rotor.OFFSET_0_FRONT + Rotor.ENCODER_OFFSET));

            telemetry.addLine(HOOD.markIf(selected) + HOOD.name() + " at " + (hoodMax ? "max" : "min"));
            telemetry.addLine(ROTOR.markIf(selected) + ROTOR.name());
            telemetry.update();
        }
    }
}
