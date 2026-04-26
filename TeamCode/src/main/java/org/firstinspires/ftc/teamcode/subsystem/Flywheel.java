package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.control.Ranges.clip;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.KalmanFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.KalmanGains;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

@Config
public final class Flywheel {

    public static PIDGains pidGains = new PIDGains(0.0005, 0, 0, 1);
    public static KalmanGains
            rpmFilterGains = new KalmanGains(1, 2),
            derivFilterGains = new KalmanGains(3, 0);

    public static double

            MAX_VOLTAGE = 13,

            RPM_ARMING = 2700,
            RPM_IDLE = 0,

            RPM_TOLERANCE = 80, // TODO increase for faster feeding

            CACHE_THRESHOLD_MOTORS = 0.001,

            RPM_PER_IN_PER_SEC = 22.1835942324; // https://www.desmos.com/calculator/2prs6sixtf

    private final CachedMotorEx[] motors;
    private final VoltageSensor batteryVoltageSensor;

    private final KalmanFilter rpmFilter = new KalmanFilter(rpmFilterGains), derivFilter = new KalmanFilter(derivFilterGains);
    private final PIDController controller = new PIDController(derivFilter);
    private final State setpoint = new State(), measurement = new State();

    private double currentRPM, targetRPM = RPM_ARMING, rawRPM, output;

    void setRPM(double rpm) {
        this.targetRPM = rpm;
    }
    static double getRPMDrop(double preLaunchRPM) {
        return 0.271632 * preLaunchRPM + 109.1459;
    }
    
    void setVelocity(double inPerSec) {
        setRPM(inPerSec * RPM_PER_IN_PER_SEC);
    }
    double getVelocity() {
        return currentRPM / RPM_PER_IN_PER_SEC;
    }

    private double manualPower;
    public void setManual(double power) {
        manualPower = power;
    }

    Flywheel(HardwareMap hardwareMap) {

        motors = new CachedMotorEx[]{
                new CachedMotorEx(hardwareMap, "shooter R", Motor.GoBILDA.BARE),
                new CachedMotorEx(hardwareMap, "shooter L", Motor.GoBILDA.BARE)
        };
        motors[1].setInverted(true);
        for (CachedMotorEx motor : motors)
            motor.setZeroPowerBehavior(FLOAT);

        motors[0].encoder = new CachedMotorEx(hardwareMap, "BR", Motor.GoBILDA.BARE).encoder;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    void run(boolean inLaunchZone, boolean feedsPending) {
        rpmFilter.setGains(rpmFilterGains);
        derivFilter.setGains(derivFilterGains);
        controller.setGains(pidGains);

        rawRPM = motors[0].encoder.getCorrectedVelocity() * 60 / 28.0 * 1.35;

        currentRPM = rpmFilter.calculate(rawRPM);

        double voltageScalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();

        double rpmSetpoint =
                !feedsPending ? RPM_IDLE :
                !inLaunchZone ? RPM_ARMING :
                                targetRPM;

        controller.setTarget(setpoint.set(rpmSetpoint));
        double pidf = controller.calculate(measurement.set(currentRPM)) // pid
                + getFeedForward(rpmSetpoint) * voltageScalar; // feedforward

        output =
                manualPower != 0 ?  manualPower :
                rpmSetpoint == 0 ? 0 :
                clip(pidf, 0, 1);


        for (CachedMotorEx motor : motors) {
            motor.threshold = CACHE_THRESHOLD_MOTORS;
            motor.set(output);
        }
    }

    /**
     * <a href="https://www.desmos.com/calculator/wcez9ivos3">Desmos curve fit</a>
     */
    private static double getFeedForward(double rpmSetpoint) {
        return rpmSetpoint * 0.000120292276714 + 0.0393080566445;
    }

    boolean inTolerance() {
        return abs(targetRPM - currentRPM) <= RPM_TOLERANCE;
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("SHOOTER", inTolerance() ? "RPM in feeding tolerance" : "RPM out of tolerance");
        telemetry.addLine();
        telemetry.addData("Current vel (rpm)", currentRPM);
        telemetry.addData("Target vel (rpm)", targetRPM);
        telemetry.addData("Raw vel (rpm)", rawRPM);
        telemetry.addData("Output power [0,1]", output);
        telemetry.addLine();
        telemetry.addData("Filtered error derivative (rpm/s)", controller.getFilteredErrorDerivative());
        telemetry.addData("Raw error derivative (rpm/s)", controller.getRawErrorDerivative());
    }
}
