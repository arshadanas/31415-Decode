package org.firstinspires.ftc.teamcode.subsystem.utility;

import static org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator.LEDColor.GREEN;
import static org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator.LEDColor.OFF;
import static org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator.LEDColor.RED;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDIndicator {

    public enum LEDColor {
        RED,
        GREEN,
        AMBER,
        OFF,
    }

    private final DigitalChannel redLED, greenLED;

    /**
     * @param greenName Configured name of the EVEN pin
     * @param redName   Configured name of the ODD pin
     */
    public LEDIndicator(HardwareMap hardwareMap, String greenName, String redName) {
        // Get the LED colors and touch sensor from the hardwaremap
        greenLED = hardwareMap.get(DigitalChannel.class, greenName);
        redLED = hardwareMap.get(DigitalChannel.class, redName);

        // change LED mode from input to output
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setColor(LEDColor color) {
        greenLED.setState(color == GREEN || color == OFF);
        redLED.setState(color == RED || color == OFF);
    }
}