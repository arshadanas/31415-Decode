package org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class CachedDcMotor {

    public final DcMotorSimple motor;

    public double threshold = 0;

    private double lastPower = Double.NaN;

    public CachedDcMotor(DcMotorSimple motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        if (abs(power - lastPower) > threshold)
            motor.setPower(lastPower = power);
    }
}
