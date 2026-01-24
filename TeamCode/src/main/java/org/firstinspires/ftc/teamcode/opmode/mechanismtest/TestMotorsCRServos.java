package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.control.Wrap.wrap;
import static org.firstinspires.ftc.teamcode.opmode.mechanismtest.TestMotorsCRServos.TestMech.ROTOR;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

@TeleOp(group = "Multiple mechanism test")
public final class TestMotorsCRServos extends LinearOpMode {

    enum TestMech {
        ROTOR, FEEDER, SHOOTER, TURRET, INTAKE;

        private final static TestMech[] values = values();

        public TestMech plus(int n) {
            return values[wrap(ordinal() + n, 0, values.length)];
        }

        public String markIf(TestMech s) {
            return this == s ? "> " : "  ";
        }
    }

    @Override
    public void runOpMode() {

        // pos = CCW
        CRServo[] rotorServos = {
                hardwareMap.get(CRServo.class, "rotor 1"),
                hardwareMap.get(CRServo.class, "rotor 2")
        };

        CRServo[] feederServos = {
                hardwareMap.get(CRServo.class, "feeder R"),
                hardwareMap.get(CRServo.class, "feeder L")
        };
        feederServos[0].setDirection(REVERSE);

        CachedMotorEx[] shooterMotors = {
                new CachedMotorEx(hardwareMap, "shooter R", Motor.GoBILDA.BARE),
                new CachedMotorEx(hardwareMap, "shooter L", Motor.GoBILDA.BARE)
        };
        shooterMotors[1].setInverted(true);
        for (CachedMotorEx motor : shooterMotors) {
            motor.setZeroPowerBehavior(FLOAT);
        }

        // pos = CCW
        CachedMotorEx turret = new CachedMotorEx(hardwareMap,  "turret", Motor.GoBILDA.RPM_1150);
        turret.setInverted(true);

        CachedMotorEx intake = new CachedMotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intake.setInverted(true);
        intake.setZeroPowerBehavior(FLOAT);

        TestMech selected = ROTOR;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpadUpWasPressed()) selected = selected.plus(-1);
            if (gamepad1.dpadDownWasPressed()) selected = selected.plus(1);

            if (gamepad1.square) {
                float triggersSum = gamepad1.right_trigger - gamepad1.left_trigger;

                switch (selected) {
                    case ROTOR:
                        for (CRServo servo : rotorServos)
                            servo.setPower(triggersSum);
                        break;
                    case FEEDER:
                        for (CRServo feederServo : feederServos)
                            feederServo.setPower(triggersSum);
                        break;
                    case SHOOTER:
                        for (CachedMotorEx shooterMotor : shooterMotors)
                            shooterMotor.set(triggersSum);
                        break;
                    case TURRET:
                        turret.set(triggersSum);
                        break;
                    case INTAKE:
                        intake.set(triggersSum);
                        break;
                }
            }

            for (TestMech mech : TestMech.values)
                telemetry.addLine(mech.markIf(selected) + mech.name() + (mech != selected ? "" : gamepad1.square ? " [EDITING]" : "[Hold square to edit]"));
            telemetry.update();
        }
    }
}
