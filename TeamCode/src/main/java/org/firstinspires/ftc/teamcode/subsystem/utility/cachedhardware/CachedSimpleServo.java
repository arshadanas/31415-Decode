package org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CachedSimpleServo extends SimpleServo {

    public CachedSimpleServo(HardwareMap hw, String servoName, double minDegree, double maxDegree) {
        super(hw, servoName, minDegree, maxDegree);
    }

    public CachedSimpleServo reversed() {
        setInverted(true);
        return this;
    }

    public double offset, threshold;

    private double lastDegrees = Double.NaN;

    public void turnToAngle(double degrees) {
        degrees += offset;
        if (Double.isNaN(lastDegrees) || abs(degrees - lastDegrees) > threshold)
            super.turnToAngle(lastDegrees = degrees);
    }
}
