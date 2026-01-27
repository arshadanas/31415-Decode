package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

public final class Lift {

    public static PIDGains gains = new PIDGains();

    public static double
            ANGLE_SWITCH_INACTIVE = 36,
            ANGLE_SWITCH_ENGAGED = 67,
            ANGLE_SWITCH_L_OFFSET = 4,
            TIME_GEAR_SWITCH = 2,
            HOLDING_POWER = 0.8,
            TARGET_LIFTING = 20,
            HEIGHT_HOLD = 30;

    private final CachedMotorEx[] motors;
    private final SimpleServoPivot gearSwitch;
    private final CachedSimpleServo gearL;

    private double manualPower;
    public void setManualPower(double power) {
        this.manualPower = power;
    }

    private final PIDController controller = new PIDController();

    private final ElapsedTime gearSwitchTimer = new ElapsedTime();

    Lift(HardwareMap hardwareMap) {;
        gearSwitch = new SimpleServoPivot(
                ANGLE_SWITCH_INACTIVE,
                ANGLE_SWITCH_ENGAGED,
                new CachedSimpleServo(hardwareMap, "gear R", 0, 1800 / 28.0), // 64.28571428571429
                gearL = new CachedSimpleServo(hardwareMap, "gear L", 0, 1800 / 28.0)
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

        set(false);
    }

    void run() {

        gearSwitch.updateAngles(ANGLE_SWITCH_INACTIVE, ANGLE_SWITCH_ENGAGED);
        gearL.offset = ANGLE_SWITCH_L_OFFSET;

        if (!isActive || gearSwitchTimer.seconds() <= TIME_GEAR_SWITCH)
            return;

        double position = motors[0].encoder.getPosition();

        controller.setGains(gains);
        controller.setTarget(new State(TARGET_LIFTING));

        double output =
                manualPower != 0 ? manualPower :
                position > HEIGHT_HOLD ? HOLDING_POWER :
                controller.calculate(new State(TARGET_LIFTING));

        for (CachedMotorEx motor : motors)
            motor.set(output);
    }

    boolean isActive = false;

    private void set(boolean isActive) {
        this.isActive = isActive;

        gearSwitch.setActivated(isActive);

        if (isActive) {
            gearSwitchTimer.reset();
            motors[0].encoder.reset();
        }

    }

    public void toggle() {
        set(!isActive);
    }

    void print(Telemetry telemetry) {
        telemetry.addData("LIFT", isActive ? "ACTIVE" : "inactive");
        telemetry.addLine();
        telemetry.addData("Gear switch", gearSwitch.isActivated() ? "ENGAGED" : "inactive");
    }
}
