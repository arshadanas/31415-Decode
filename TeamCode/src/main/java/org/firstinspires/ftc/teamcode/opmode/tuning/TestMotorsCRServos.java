package org.firstinspires.ftc.teamcode.opmode.tuning;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.control.Ranges.wrap;
import static org.firstinspires.ftc.teamcode.opmode.tuning.TestMotorsCRServos.TestMech.ROTOR;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

@TeleOp(group = "Testing/tuning")
public final class TestMotorsCRServos extends LinearOpMode {

    enum TestMech {
        ROTOR, FEEDER, SHOOTER, SHOOTER_R, SHOOTER_L, TURRET, INTAKE, FR, FL, BR, BL;

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

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

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

        CachedMotorEx
                FR = new CachedMotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_435),
                FL = new CachedMotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_435),
                BR = new CachedMotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_435),
                BL = new CachedMotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_435);

        FL.setInverted(true);
        BL.setInverted(true);
        BR.setRunMode(Motor.RunMode.RawPower);
        BR.setZeroPowerBehavior(FLOAT);

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
                    case SHOOTER_R:
                        shooterMotors[0].set(triggersSum);
                        break;
                    case SHOOTER_L:
                        shooterMotors[1].set(triggersSum);
                        break;
                    case TURRET:
                        turret.set(triggersSum);
                        break;
                    case INTAKE:
                        intake.set(triggersSum);
                        break;
                    case FR:
                        FR.set(triggersSum);
                        break;
                    case FL:
                        FL.set(triggersSum);
                        break;
                    case BR:
                        BR.set(triggersSum);
                        break;
                    case BL:
                        BL.set(triggersSum);
                        break;
                }
            }

            for (TestMech mech : TestMech.values)
                telemetry.addLine(mech.markIf(selected) + mech.name() + (mech != selected ? "" : gamepad1.square ? " [EDITING]" : " [Hold square to edit]"));
            telemetry.update();
        }
    }
}
