package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static org.firstinspires.ftc.teamcode.control.Wrap.wrap;
import static org.firstinspires.ftc.teamcode.opmode.mechanismtest.TuneServos.TestServo.GATE_L;
import static org.firstinspires.ftc.teamcode.opmode.mechanismtest.TuneServos.TestServo.GATE_R;
import static org.firstinspires.ftc.teamcode.opmode.mechanismtest.TuneServos.TestServo.GEAR_L;
import static org.firstinspires.ftc.teamcode.opmode.mechanismtest.TuneServos.TestServo.GEAR_R;
import static org.firstinspires.ftc.teamcode.opmode.mechanismtest.TuneServos.TestServo.HOOD;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Configurable
@TeleOp(group = "Multiple mechanism test")
public final class TuneServos extends LinearOpMode {

    public static double
            ANGLE_HOOD_MIN = 0,
            ANGLE_HOOD_MAX = 360,

            ANGLE_GATE_R_MIN = 0,
            ANGLE_GATE_R_MAX = 300,

            ANGLE_GATE_L_MIN = 0,
            ANGLE_GATE_L_MAX = 300,

            ANGLE_GEAR_R_MIN = 0,
            ANGLE_GEAR_R_MAX = 1800 / 28.0,

            ANGLE_GEAR_L_MIN = 0,
            ANGLE_GEAR_L_MAX = 1800 / 28.0;

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

        CachedSimpleServo hood = new CachedSimpleServo(hardwareMap, "hood", 0, 360);
        CachedSimpleServo gateR = new CachedSimpleServo(hardwareMap, "gate R", 0, 300);
        CachedSimpleServo gateL = new CachedSimpleServo(hardwareMap, "gate L", 0, 300);
        CachedSimpleServo gearR = new CachedSimpleServo(hardwareMap, "gear R", 0, 1800 / 28.0);
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

            hood.turnToAngle(hoodMax ? ANGLE_HOOD_MAX : ANGLE_HOOD_MIN);
            gateR.turnToAngle(gateRMax ? ANGLE_GATE_R_MAX : ANGLE_GATE_R_MIN);
            gateL.turnToAngle(gateLMax ? ANGLE_GATE_L_MAX : ANGLE_GATE_L_MIN);
            gearR.turnToAngle(gearRMax ? ANGLE_GEAR_R_MAX : ANGLE_GEAR_R_MIN);
            gearL.turnToAngle(gearLMax ? ANGLE_GEAR_L_MAX : ANGLE_GEAR_L_MIN);

            telemetry.addLine(HOOD.markIf(selected) + HOOD.name() + " at " + (hoodMax ? "max" : "min"));
            telemetry.addLine(GATE_R.markIf(selected) + GATE_R.name() + " at " + (gateRMax ? "max" : "min"));
            telemetry.addLine(GATE_L.markIf(selected) + GATE_L.name() + " at " + (gateLMax ? "max" : "min"));
            telemetry.addLine(GEAR_R.markIf(selected) + GEAR_R.name() + " at " + (gearRMax ? "max" : "min"));
            telemetry.addLine(GEAR_L.markIf(selected) + GEAR_L.name() + " at " + (gearLMax ? "max" : "min"));
            telemetry.update();
        }
    }
}
