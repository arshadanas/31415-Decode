package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Lift {

    public static double
            ANGLE_SWITCH_INACTIVE = 50,
            ANGLE_SWITCH_ENGAGED = 67,
            ANGLE_SWITCH_L_OFFSET = 4;

    private final CachedMotorEx[] motors;

    private final CachedSimpleServo[] gearSwitch;

    Lift(HardwareMap hardwareMap) {
        gearSwitch = new CachedSimpleServo[]{
                new CachedSimpleServo(hardwareMap, "gear R", 0, 1800 / 28.0), // 64.28571428571429
                new CachedSimpleServo(hardwareMap, "gear L", 0, 1800 / 28.0),
        };
        gearSwitch[1].offset = ANGLE_SWITCH_L_OFFSET;

        motors = new CachedMotorEx[]{
                new CachedMotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_435),
                new CachedMotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_435),
                new CachedMotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_435),
                new CachedMotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_435),
        };
        motors[1].setInverted(true);
        motors[3].setInverted(true);
    }

    void run() {
        gearSwitch[1].offset = ANGLE_SWITCH_L_OFFSET;

        for (CachedSimpleServo servo : gearSwitch)
            servo.turnToAngle(enabled ? ANGLE_SWITCH_ENGAGED : ANGLE_SWITCH_INACTIVE);

        if (enabled)
            for (CachedMotorEx motor : motors)
                motor.set(manualPower);
    }

    private double manualPower;
    public void setManualPower(double power) {
        this.manualPower = power;
    }

    public boolean enabled = false;
}
