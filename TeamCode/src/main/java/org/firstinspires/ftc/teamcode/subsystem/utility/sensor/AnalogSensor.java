package org.firstinspires.ftc.teamcode.subsystem.utility.sensor;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AnalogSensor {

    private final AnalogInput sensor;

    private final double maxOutput;

    public AnalogSensor(HardwareMap hardwareMap, String encoderName, double maxOutput) {

        sensor = hardwareMap.get(AnalogInput.class, encoderName);

        this.maxOutput = maxOutput;
    }

    public double getReading() {
        return maxOutput * sensor.getVoltage() / sensor.getMaxVoltage();
    }
}
