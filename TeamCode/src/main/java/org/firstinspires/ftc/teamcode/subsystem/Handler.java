package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

public final class Handler {

    public final Container container;
    private final CachedMotorEx intake;
    private final CRServo[] feeder;

    private double intakePower;
    public void setIntake(double power) {
        this.intakePower = power;
    }

    private double feederPower;
    public void setFeeder(double power) {
        this.feederPower = power;
    }

    Handler(HardwareMap hardwareMap) {

        container = new Container(hardwareMap);

        intake = new CachedMotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intake.setInverted(true);
        intake.setZeroPowerBehavior(FLOAT);

        feeder = new CRServo[]{
                hardwareMap.get(CRServo.class, "feeder R"),
                hardwareMap.get(CRServo.class, "feeder L")
        };
        feeder[0].setDirection(REVERSE);
    }

    void run() {

        for (CRServo servo : feeder)
            servo.setPower(feederPower);

        container.run(feederPower);

        intake.set(
                intakePower < 0 ? intakePower :
                max(intakePower, container.getMinIntakeSpeed())
        );

    }

    void print(Telemetry telemetry) {
        container.print(telemetry);
    }

}
