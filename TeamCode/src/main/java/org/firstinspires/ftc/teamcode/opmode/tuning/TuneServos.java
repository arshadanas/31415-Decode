package org.firstinspires.ftc.teamcode.opmode.tuning;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.control.Ranges.wrap;
import static org.firstinspires.ftc.teamcode.opmode.tuning.TuneServos.TestServo.GATE_L;
import static org.firstinspires.ftc.teamcode.opmode.tuning.TuneServos.TestServo.GATE_R;
import static org.firstinspires.ftc.teamcode.opmode.tuning.TuneServos.TestServo.GEAR_L;
import static org.firstinspires.ftc.teamcode.opmode.tuning.TuneServos.TestServo.GEAR_R;
import static org.firstinspires.ftc.teamcode.opmode.tuning.TuneServos.TestServo.HOOD;
import static org.firstinspires.ftc.teamcode.opmode.tuning.TuneServos.TestServo.ROTOR;
import static org.firstinspires.ftc.teamcode.subsystem.Handler.ANGLE_PRESSER_EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystem.Handler.ANGLE_PRESSER_L_OFFSET;
import static org.firstinspires.ftc.teamcode.subsystem.Handler.ANGLE_PRESSER_RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ANGLE_SWITCH_ENGAGED;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ANGLE_SWITCH_INACTIVE;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ANGLE_SWITCH_L_OFFSET;
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

    public static double rotorAngle, rotor2offset;

    enum TestServo {
        HOOD, GATE_R, GATE_L, GEAR_R, GEAR_L, ROTOR;

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
        CachedSimpleServo gateR = new CachedSimpleServo(hardwareMap, "gate R", 0, 300);
        CachedSimpleServo gateL = new CachedSimpleServo(hardwareMap, "gate L", 0, 300).reversed();
        CachedSimpleServo gearR = new CachedSimpleServo(hardwareMap, "gear R", 0, 1800 / 28.0); // 64.28571428571429
        CachedSimpleServo gearL = new CachedSimpleServo(hardwareMap, "gear L", 0, 1800 / 28.0);
        CachedSimpleServo rotor1 = new CachedSimpleServo(hardwareMap, "rotor 1", -PI, PI);
        CachedSimpleServo rotor2 = new CachedSimpleServo(hardwareMap, "rotor 2", -PI, PI);

        boolean
                hoodMax = false,
                gateRMax = false,
                gateLMax = false,
                gearRMax = false,
                gearLMax = false;

        TestServo selected = HOOD;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpadUpWasPressed()) selected = selected.plus(-1);
            if (gamepad1.dpadDownWasPressed()) selected = selected.plus(1);

            if (gamepad1.squareWasPressed()) {
                switch (selected) {
                    case HOOD:
                        hoodMax = !hoodMax;
                        break;
                        
                    case GATE_R:
                        gateRMax = !gateRMax;
                        break;
                        
                    case GATE_L:
                        gateLMax = !gateLMax;
                        break;
                        
                    case GEAR_R:
                        gearRMax = !gearRMax;
                        break;
                        
                    case GEAR_L:
                        gearLMax = !gearLMax;
                        break;

                    case ROTOR:
                        rotorAngle = normalizeRadians(rotorAngle + PI / 3);
                        rotorAngle = normalizeRadians(rotorAngle - PI / 3);
                        break;
                        
                }
            }

            hood.turnToAngle(hoodMax ? ANGLE_HOOD_SHALLOWEST : ANGLE_HOOD_STEEPEST);

            gateR.turnToAngle(gateRMax ? ANGLE_PRESSER_EXTENDED : ANGLE_PRESSER_RETRACTED);
            gearL.offset = ANGLE_SWITCH_L_OFFSET;
            gateL.turnToAngle(gateLMax ? ANGLE_PRESSER_EXTENDED : ANGLE_PRESSER_RETRACTED);

            gearR.turnToAngle(gearRMax ? ANGLE_SWITCH_ENGAGED : ANGLE_SWITCH_INACTIVE);
            gateL.offset = ANGLE_PRESSER_L_OFFSET;
            gearL.turnToAngle(gearLMax ? ANGLE_SWITCH_ENGAGED : ANGLE_SWITCH_INACTIVE);

            rotor1.turnToAngle(normalizeRadians(rotorAngle + Rotor.ROTOR_OUTPUT_OFFSET));
            rotor2.turnToAngle(normalizeRadians(rotorAngle + Rotor.ROTOR_OUTPUT_OFFSET + rotor2offset));

            telemetry.addLine(HOOD.markIf(selected) + HOOD.name() + " at " + (hoodMax ? "max" : "min"));
            telemetry.addLine(GATE_R.markIf(selected) + GATE_R.name() + " at " + (gateRMax ? "max" : "min"));
            telemetry.addLine(GATE_L.markIf(selected) + GATE_L.name() + " at " + (gateLMax ? "max" : "min"));
            telemetry.addLine(GEAR_R.markIf(selected) + GEAR_R.name() + " at " + (gearRMax ? "max" : "min"));
            telemetry.addLine(GEAR_L.markIf(selected) + GEAR_L.name() + " at " + (gearLMax ? "max" : "min"));
            telemetry.addLine(ROTOR.markIf(selected) + ROTOR.name());
            telemetry.update();
        }
    }
}
