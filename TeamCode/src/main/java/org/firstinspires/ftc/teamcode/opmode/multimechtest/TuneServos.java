package org.firstinspires.ftc.teamcode.opmode.multimechtest;

import static com.acmerobotics.roadrunner.Math.lerp;
import static org.firstinspires.ftc.teamcode.control.Wrap.wrap;
import static org.firstinspires.ftc.teamcode.opmode.multimechtest.TuneServos.TestServo.GATE_L;
import static org.firstinspires.ftc.teamcode.opmode.multimechtest.TuneServos.TestServo.GATE_R;
import static org.firstinspires.ftc.teamcode.opmode.multimechtest.TuneServos.TestServo.GEAR_L;
import static org.firstinspires.ftc.teamcode.opmode.multimechtest.TuneServos.TestServo.GEAR_R;
import static org.firstinspires.ftc.teamcode.opmode.multimechtest.TuneServos.TestServo.HOOD;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ANGLE_SWITCH_ENGAGED;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ANGLE_SWITCH_INACTIVE;
import static org.firstinspires.ftc.teamcode.subsystem.Lift.ANGLE_SWITCH_L_OFFSET;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
@TeleOp(group = "Multiple mechanism test")
public final class TuneServos extends LinearOpMode {

    public static double

            LAUNCH_DEG_SHALLOWEST = 31.901328,
            LAUNCH_DEG_STEEPEST = 61.7419355,

            ANGLE_HOOD_SERVO_MAX = 360,
            ANGLE_HOOD_SERVO_MIN = 10,

            ANGLE_PRESSER_RETRACTED = 87,
            ANGLE_PRESSER_EXTENDED = 211,
            ANGLE_PRESSER_L_OFFSET = -37;

    public static double launchDegToServoDeg(double launchDegrees) {
        return lerp(launchDegrees, LAUNCH_DEG_SHALLOWEST, LAUNCH_DEG_STEEPEST, ANGLE_HOOD_SERVO_MAX, ANGLE_HOOD_SERVO_MIN);
    }

    enum TestServo {
        HOOD, GATE_R, GATE_L, GEAR_R, GEAR_L;

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
                        
                }
            }

            hood.turnToAngle(hoodMax ? ANGLE_HOOD_SERVO_MAX : ANGLE_HOOD_SERVO_MIN);

            gateR.turnToAngle(gateRMax ? ANGLE_PRESSER_EXTENDED : ANGLE_PRESSER_RETRACTED);
            gearL.offset = ANGLE_SWITCH_L_OFFSET;
            gateL.turnToAngle(gateLMax ? ANGLE_PRESSER_EXTENDED : ANGLE_PRESSER_RETRACTED);

            gearR.turnToAngle(gearRMax ? ANGLE_SWITCH_ENGAGED : ANGLE_SWITCH_INACTIVE);
            gateL.offset = ANGLE_PRESSER_L_OFFSET;
            gearL.turnToAngle(gearLMax ? ANGLE_SWITCH_ENGAGED : ANGLE_SWITCH_INACTIVE);

            telemetry.addLine(HOOD.markIf(selected) + HOOD.name() + " at " + (hoodMax ? "max" : "min"));
            telemetry.addLine(GATE_R.markIf(selected) + GATE_R.name() + " at " + (gateRMax ? "max" : "min"));
            telemetry.addLine(GATE_L.markIf(selected) + GATE_L.name() + " at " + (gateLMax ? "max" : "min"));
            telemetry.addLine(GEAR_R.markIf(selected) + GEAR_R.name() + " at " + (gearRMax ? "max" : "min"));
            telemetry.addLine(GEAR_L.markIf(selected) + GEAR_L.name() + " at " + (gearLMax ? "max" : "min"));
            telemetry.update();
        }
    }
}
