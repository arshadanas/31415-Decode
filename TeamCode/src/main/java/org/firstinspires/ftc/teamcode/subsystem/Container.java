package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.control.Ranges.lerp;
import static org.firstinspires.ftc.teamcode.control.Ranges.wrap;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.KalmanFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.KalmanGains;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedDcMotor;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

import java.util.Arrays;

@Config
public final class Container {

    private final KalmanFilter derivFilter = new KalmanFilter(filterGains, true);
    public static KalmanGains filterGains = new KalmanGains(500, 3000);

    private final Differentiator kD = new Differentiator();

    private final PIDController controller = new PIDController();
    public static PIDGains
            pidGainsEmpty = new PIDGains(0.125, 0.1, 0.01, 0.1),
            pidGainsFull = new PIDGains(0.125, 0.1, 0.01, 0.1);

    private final PIDGains pidGains = new PIDGains();

    private PIDGains getPIDGains() {
        double t = EMPTY.numOccurrencesIn(artifacts) / 3.0;
        pidGains.kP = lerp(
                pidGainsFull.kP,
                pidGainsEmpty.kP,
                t
        );
        pidGains.kI = lerp(
                pidGainsFull.kI,
                pidGainsEmpty.kI,
                t
        );
        pidGains.kD = lerp(
                pidGainsFull.kD,
                pidGainsEmpty.kD,
                t
        );
        pidGains.maxOutputWithIntegral = lerp(
                pidGainsFull.maxOutputWithIntegral,
                pidGainsEmpty.maxOutputWithIntegral,
                t
        );
        return pidGains;
    }

    public static double
            ABS_OFFSET_ROTOR = 2.882649259112089,
            THRESHOLD_FRONT_MM = 95, // start of ramp = ~115
            THRESHOLD_BACK_MM = 70, // Height to move onto next feed; above rotor = ~75 // TODO Decrease for faster feeding
            TIME_BACK_DIST = 0.125,
            INTAKE_POWER_OMNI_CONTACT = 0.4,
            INTAKE_POWER_IDLE = 0,

            TOLERANCE_INTAKE_SENSORS = 0.2, // too high => false positives, too low => false negatives (no-detect)
            TOLERANCE_FEEDER_SENSORS = 0.15, // too high => false negatives (removals)
            TOLERANCE_FEEDER_OMNIS = 0.6108652381980153,
            TOLERANCE_FEEDER_FRICTION = 0.6108652381980153,
            TOLERANCE_INTAKE_OMNI = 0.5235987755982988,

            POWER_OVERCOME_FRICTION = 0.06,
            MAX_VOLTAGE = 13,

            CACHE_THRESHOLD_ROTOR = 0.05;

    // hardware
    private final CachedDcMotor[] servos;
    private final AnalogSensor encoder, front1, back1;
    private final ColorSensor color1, color2;
//    private final LEDIndicator[] indicators;
    private final VoltageSensor batteryVoltageSensor;

    private final ElapsedTime backDistanceTimer = new ElapsedTime();

    /**
     * Position of slot 0, in radians
     */
    private double position = 0;

    /**
     * Position of given slot, in radians
     */
    private double getPositionOf(int slot) {
        return normalizeRadians(position + wrap(slot, 0, artifacts.length) * 2 * PI / 3.0);
    }

    final Artifact[] artifacts = {EMPTY, EMPTY, EMPTY};

    private int selectedSlot = 0;
    private Zone target = Zone.INTAKE_SENSORS;

    enum Zone {
        INTAKE_SENSORS(0),
        INTAKE_OMNI(0),
        FEEDER_SENSORS(PI),
        FEEDER_OMNIS(PI),
        FEEDER_FRICTION(PI);

        private final double radians;
        Zone(double radians) {
            this.radians = radians;
        }

        private double getTolerance() {
            switch (this) {
                case INTAKE_OMNI:       return TOLERANCE_INTAKE_OMNI;
                case FEEDER_SENSORS:    return TOLERANCE_FEEDER_SENSORS;
                case FEEDER_OMNIS:      return TOLERANCE_FEEDER_OMNIS;
                case FEEDER_FRICTION:   return TOLERANCE_FEEDER_FRICTION;
                default:                return TOLERANCE_INTAKE_SENSORS;
            }
        }

    }

    Container(HardwareMap hardwareMap) {
        servos = new CachedDcMotor[]{
                new CachedDcMotor(hardwareMap.get(CRServo.class, "rotor 1")),
                new CachedDcMotor(hardwareMap.get(CRServo.class, "rotor 2"))
        };

        encoder = new AnalogSensor(hardwareMap, "rotor", 2 * PI);

        front1 = new AnalogSensor(hardwareMap, "front 1", 1300);
        back1 = new AnalogSensor(hardwareMap, "back 1", 1000);

        color1 = new ColorSensor(hardwareMap, "color 1", 1);
        color2 = new ColorSensor(hardwareMap, "color 2", 1);

//        indicators = new LEDIndicator[]{
//                new LEDIndicator(hardwareMap, "led 1a", "led 1b"),
//                new LEDIndicator(hardwareMap, "led 2a", "led 2b"),
//                new LEDIndicator(hardwareMap, "led 3a", "led 3b")
//        };

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    void run(double intakePower, double feederPower) {
        double oldPos = position;
        position = normalizeRadians(encoder.getReading() + ABS_OFFSET_ROTOR);
        double velocity = kD.getDerivative(
                position == oldPos ? derivFilter.x_previous : derivFilter.calculate(position));

        int currentFrontSlot = getSlotAt(Zone.INTAKE_SENSORS);
        if (
                intakePower > 0 && // intake is running
                currentFrontSlot != -1 &&   // there is a slot near the front intaking zone
                artifacts[currentFrontSlot] == EMPTY && // the slot was previously empty
                front1.getReading() < THRESHOLD_FRONT_MM // there is something in front of the distance sensor
        ) {
            // read i2c
            color1.update();
            color2.update();

            // combine Artifact reading from both color sensors
            artifacts[currentFrontSlot] = Artifact.fromHSV(color1.getHSV()). or (Artifact.fromHSV(color2.getHSV()));
            updateLEDs();
        }


        // check back slot sensors
        int currentBackSlot = getSlotAt(Zone.FEEDER_SENSORS);
        if (
                feederPower > 0 &&  // the feeder is running
                currentBackSlot != -1 && // there is a slot near the back feeding zone
                artifacts[currentBackSlot] != EMPTY && // the slot was not previously empty
                back1.getReading() > THRESHOLD_BACK_MM // distance sensor reports no artifact
        ) {
            if (backDistanceTimer.seconds() >= TIME_BACK_DIST) {
                artifacts[currentBackSlot] = EMPTY; // clear the back slot since it has been fed out
                updateLEDs();
            }
        } else backDistanceTimer.reset();

        // run pid
        derivFilter.setGains(filterGains);
        controller.setGains(getPIDGains());
        controller.setTarget(new State(getError(selectedSlot, target)));

        double servoPower = controller.calculate(new State(0, velocity));

        double voltageScalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();

        int frictionSlot = getSlotAt(Zone.FEEDER_FRICTION);
        double antiFrictionPower = frictionSlot != -1 && artifacts[frictionSlot] != EMPTY ?
                                    POWER_OVERCOME_FRICTION * signum(servoPower) * voltageScalar : 0;

        for (CachedDcMotor servo : servos) {
            servo.threshold = CACHE_THRESHOLD_ROTOR;
            servo.setPower(servoPower + antiFrictionPower);
        }
    }

    void updateLEDs() {
//        int n = 0;
//        for (Artifact a : artifacts)
//            if (a != EMPTY)
//                indicators[n++].setColor(a == GREEN ? LEDIndicator.LEDColor.GREEN : LEDIndicator.LEDColor.RED);
//        while (n < artifacts.length)
//            indicators[n++].setColor(LEDIndicator.LEDColor.OFF);
    }

    /**
     * Rounds intake speeds of [0, {@link #INTAKE_POWER_OMNI_CONTACT}) up to {@link #INTAKE_POWER_OMNI_CONTACT}
     * if there is an {@link Artifact} touching the intake's front omni wheel <br><br>
     * Rounds speeds of ({@link #INTAKE_POWER_IDLE}, 0] down to {@link #INTAKE_POWER_IDLE}
     * if there is no {@link Artifact} touching the intake's front omni wheel
     */
    double adaptiveClipIntakePower(double intakePower) {
        int omniSlot = getSlotAt(Zone.INTAKE_OMNI);
        if (
                intakePower >= 0 &&
                intakePower < INTAKE_POWER_OMNI_CONTACT &&
                omniSlot != -1 && artifacts[omniSlot] != EMPTY // artifact touching omni wheel
        )
            return INTAKE_POWER_OMNI_CONTACT;

        if (
                intakePower <= 0 &&
                intakePower > INTAKE_POWER_IDLE &&
                (omniSlot == -1 || artifacts[omniSlot] == EMPTY) // no artifact near the omni wheel
        )
            return INTAKE_POWER_IDLE;

        return intakePower;
    }

    /**
     * @param slot Slot you wish to move (0, 1 or 2)
     */
    void moveSlot(int slot, Zone target) {
        this.selectedSlot = wrap(slot, 0, artifacts.length);
        this.target = target;
    }

    /**
     * @return Empty slot closest to the intake. -1 if no such slot found
     */
    int getNearestIntakeSlot() {
        double min = Double.MAX_VALUE;
        int minInd = -1;
        for (int i = 0; i < 3; i++) {
            if (artifacts[i] != EMPTY)
                continue;

            double error = abs(getError(i, Zone.INTAKE_SENSORS));
            if (error < min){
                min = error;
                minInd = i;
            }
        }
        return minInd;
    }

    /**
     * @return Occupied slot closest to the feeder. -1 if no such slot found
     */
    int getNearestFeedSlot() {
        double min = Double.MAX_VALUE;
        int minInd = -1;
        for (int i = 0; i < 3; i++) {
            if (artifacts[i] == EMPTY)
                continue;

            double error = abs(getError(i, Zone.FEEDER_SENSORS));
            if (error < min){
                min = error;
                minInd = i;
            }
        }
        return minInd;
    }

    /**
     * @return Slot closest to the feeder, and holding an Artifact of the given color. -1 if no such slot found
     */
    int getNearestFeedSlot(Artifact color) {
        double min = Double.MAX_VALUE;
        int minInd = -1;
        for (int i = 0; i < 3; i++) {
            if (artifacts[i] != color)
                continue;

            double error = abs(getError(i, Zone.FEEDER_SENSORS));
            if (error < min){
                min = error;
                minInd = i;
            }
        }
        return minInd;
    }

    Artifact get(int slot) {
        return artifacts[wrap(slot, 0, artifacts.length)];
    }

    /**
     * @return The (index of the) slot currently at the given target, -1 if no slot at that position
     */
    int getSlotAt(Zone target) {
        for (int i = 0; i < artifacts.length; i++)
            if (atPosition(i, target))
                return i;
        return -1;
    }

    /**
     * @return  If the given slot is at the given target, within tolerance in either direction
     */
    boolean atPosition(int slot, Zone target) {
        return abs(getError(slot, target)) <= target.getTolerance();
    }

    /**
     * @return  Distance, in radians, between given slot's position and given target
     */
    double getError(int slot, Zone target) {
        return normalizeRadians(target.radians - getPositionOf(slot));
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("CONTAINER", Arrays.toString(artifacts));
        telemetry.addLine();
        telemetry.addData("Position [0] (deg)", toDegrees(position));
        telemetry.addData("Rotor error (deg)", toDegrees(getError(selectedSlot, target)));
        telemetry.addData("Filtered error derivative (deg/s)", toDegrees(controller.getFilteredErrorDerivative()));
        telemetry.addData("Raw error derivative (deg/s)", toDegrees(controller.getRawErrorDerivative()));
        telemetry.addLine();
        telemetry.addData("Front dist (mm)", front1.getReading());
        telemetry.addData("Back dist (mm)", back1.getReading());
    }

}
