package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Lift {

    public static double
            ANGLE_SWITCH_INACTIVE = 50,
            ANGLE_SWITCH_ENGAGED = 67,
            ANGLE_SWITCH_L_OFFSET = 4,
            HOLDING_POWER = .8,
            LIFTING_POWER = 2,
            HEIGHT_LIFTED = 16,
            TIME_GEAR_SWITCH = 1,
            INCHES_PER_TICK = 2.8694862032788664 / 384.5397923875433; // circumference of retract spool (in inches) / CPR

    private final CachedMotorEx[] motors;
    public final SimpleServoPivot gearSwitch;
    private final CachedSimpleServo switchL;

    private final ElapsedTime switchTimer = new ElapsedTime();

    Lift(HardwareMap hardwareMap) {
        gearSwitch = new SimpleServoPivot(
                ANGLE_SWITCH_INACTIVE,
                ANGLE_SWITCH_ENGAGED,
                new CachedSimpleServo(hardwareMap, "gear R", 0, 1800 / 28.0), // 64.28571428571429
                switchL = new CachedSimpleServo(hardwareMap, "gear L", 0, 1800 / 28.0)
        );

        motors = new CachedMotorEx[]{
                new CachedMotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_435),
                new CachedMotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_435),
                new CachedMotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_435),
                new CachedMotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_435),
        };
        motors[1].setInverted(true);
        motors[3].setInverted(true);

        motors[0].encoder.setDirection(REVERSE);
        motors[0].encoder.setDistancePerPulse(INCHES_PER_TICK);
    }

    void run() {

        switchL.offset = ANGLE_SWITCH_L_OFFSET;
        gearSwitch.updateAngles(ANGLE_SWITCH_INACTIVE, ANGLE_SWITCH_ENGAGED);
        gearSwitch.run();

        if (!gearSwitch.isActivated())
            switchTimer.reset();

        if (switchTimer.seconds() < TIME_GEAR_SWITCH) {
            motors[0].encoder.reset();
            setPower(LIFTING_POWER);
            return;
        }

        if (motorPower == LIFTING_POWER && motors[0].encoder.getDistance() > HEIGHT_LIFTED)
            setPower(HOLDING_POWER);

        for (CachedMotorEx motor : motors)
            motor.set(motorPower);
    }

    private double motorPower;
    public void setPower(double power) {
        this.motorPower = power;
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("LIFT",
                motorPower == LIFTING_POWER ? "LIFTING" :
                motorPower == HOLDING_POWER ?       "HOLDING POSITION" :
                motorPower != 0 ?                   "MANUAL" :
                                                    "inactive"
        );
        telemetry.addLine();
        telemetry.addData("Gear switch", switchTimer.seconds() >= TIME_GEAR_SWITCH ? "ENGAGED" : gearSwitch.isActivated() ? "MOVING" : "inactive");
        telemetry.addLine();
        telemetry.addData("Position (in)", motors[0].encoder.getPosition());
    }
}
