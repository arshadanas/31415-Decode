package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.teamcode.opmode.mechanismtest.TestMotorsCRServos.TestMech.ROTOR;
import static org.firstinspires.ftc.teamcode.control.Wrap.wrap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "Single mechanism test")
public final class TestMotorsCRServos extends LinearOpMode {

    enum TestMech {
        ROTOR, FEEDER, SHOOTER, TURRET, INTAKE;

        private final static TestMech[] values = values();

        public TestMech plus(int n) {
            return values[wrap(ordinal() + n, 0, values.length)];
        }

        public String markIf(TestMech s) {
            return this == s ? " <" : "";
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

        DcMotorEx[] shooterMotors = {
                hardwareMap.get(DcMotorEx.class, "shooter R"),
                hardwareMap.get(DcMotorEx.class, "shooter L")
        };
        shooterMotors[1].setDirection(REVERSE);
        for (DcMotorEx motor : shooterMotors) {
            motor.setZeroPowerBehavior(FLOAT);
        }

        // pos = CCW
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(REVERSE);

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(REVERSE);
        intake.setZeroPowerBehavior(FLOAT);

        TestMech testMech = ROTOR;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpadUpWasPressed()) testMech = testMech.plus(-1);
            if (gamepad1.dpadDownWasPressed()) testMech = testMech.plus(1);

            float triggersSum = gamepad1.right_trigger - gamepad1.left_trigger;

            switch (testMech) {
                case ROTOR:
                    for (CRServo servo : rotorServos)
                        servo.setPower(triggersSum);
                    break;
                case FEEDER:
                    for (CRServo feederServo : feederServos)
                        feederServo.setPower(triggersSum);
                    break;
                case SHOOTER:
                    for (DcMotorEx shooterMotor : shooterMotors)
                        shooterMotor.setPower(triggersSum);
                    break;
                case TURRET:
                    turret.setPower(triggersSum);
                    break;
                case INTAKE:
                    intake.setPower(triggersSum);
                    break;
            }

            for (TestMech mech : TestMech.values)
                telemetry.addLine(mech.name() + testMech.markIf(mech));
            telemetry.update();
        }
    }
}
