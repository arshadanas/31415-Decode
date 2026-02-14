package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.control.Ranges.wrap;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.PURPLE;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

import java.util.Arrays;

@Config
public final class Container {

    public static double
            ROTOR_ENCODER_OFFSET = -1.5742869852988852,
            ROTOR_OUTPUT_OFFSET = -1.7,
            OFFSET_0_BACK = 3.45,
            OFFSET_1_FRONT = -1.7,
            OFFSET_1_BACK = 1.175,
            OFFSET_2_FRONT = 2.35,
            OFFSET_2_BACK = -1.125,
            THRESHOLD_FRONT_MM = 70, // start of ramp = ~115
            THRESHOLD_BACK_MM = 70, // Height to move onto next feed; above rotor = ~75 // TODO Decrease for faster feeding
            TIME_BACK_DIST = 0.125,
            INTAKE_POWER_OMNI_CONTACT = 0.4,
            INTAKE_POWER_IDLE = 0,

            TIME_FRONT_DIST_RESET = 0.05,

            TIME_WRAPAROUND = 0.03,

            TOLERANCE_INTAKE_SENSORS_DEG = 11.46, // too high => false positives, too low => false negatives (no-detect)
            TOLERANCE_FEEDER_SENSORS_DEG = 8.6, // too high => false negatives (removals)
            TOLERANCE_FEEDER_OMNIS_DEG = 30,
            TOLERANCE_INTAKE_OMNI_DEG = 30;

    // hardware
    private final CachedSimpleServo servo;
    private final AnalogSensor encoder, front1, back1;
    private final ColorSensor color1, color2;
//    private final LEDIndicator[] indicators;

    private final ElapsedTime backDistanceTimer = new ElapsedTime(), frontDistanceTimer = new ElapsedTime();

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

    public final Artifact[] artifacts = {EMPTY, EMPTY, EMPTY};
    public void setArtifacts(Artifact[] artifacts) {
        assert artifacts.length == 3;
        if (Arrays.equals(this.artifacts, artifacts))
            return;

        System.arraycopy(artifacts, 0, this.artifacts, 0, 3);
        updateLEDs();
        genFeedingOrder.run();
    }

    private Artifact a1, a2;
    private HSV hsv1 = new HSV(), hsv2 = new HSV();

    private final Runnable genFeedingOrder;

    enum Zone {
        INTAKE_SENSORS(0),
        INTAKE_OMNI(0),
        FEEDER_SENSORS(PI),
        FEEDER_OMNIS(PI);

        private final double radians;
        Zone(double radians) {
            this.radians = radians;
        }

        private double getTolerance() {
            switch (this) {
                case INTAKE_OMNI:       return toRadians(TOLERANCE_INTAKE_OMNI_DEG);
                case FEEDER_SENSORS:    return toRadians(TOLERANCE_FEEDER_SENSORS_DEG);
                case FEEDER_OMNIS:      return toRadians(TOLERANCE_FEEDER_OMNIS_DEG);
                default:                return toRadians(TOLERANCE_INTAKE_SENSORS_DEG);
            }
        }

    }

    Container(HardwareMap hardwareMap, Runnable genFeedingOrder) {
        servo = new CachedSimpleServo(hardwareMap, "rotor 2", -PI, PI);

        encoder = new AnalogSensor(hardwareMap, "rotor", 2 * PI);

        front1 = new AnalogSensor(hardwareMap, "front 1", 4000);
        back1 = new AnalogSensor(hardwareMap, "back 1", 1000);

        color1 = new ColorSensor(hardwareMap, "color 1", 1);
        color2 = new ColorSensor(hardwareMap, "color 2", 1);

//        indicators = new LEDIndicator[]{
//                new LEDIndicator(hardwareMap, "led 1a", "led 1b"),
//                new LEDIndicator(hardwareMap, "led 2a", "led 2b"),
//                new LEDIndicator(hardwareMap, "led 3a", "led 3b")
//        };

        this.genFeedingOrder = genFeedingOrder;
    }

    void run(double intakePower, double feederPower) {

        position = normalizeRadians(encoder.getReading() + ROTOR_ENCODER_OFFSET);

        if (wrapAround && wrapAroundTimer.seconds() >= TIME_WRAPAROUND) {
            wrapAround = false;
            servo.turnToAngle(lastRadians);
        }

        int currentFrontSlot = getSlotAt(Zone.INTAKE_SENSORS);
        if (
                intakePower > 0 && // intake is running
                currentFrontSlot != -1 &&   // there is a slot near the front intaking zone
                artifacts[currentFrontSlot] == EMPTY && // the slot was previously empty
                front1.getReading() < THRESHOLD_FRONT_MM && // there is something in front of the distance sensor
                frontDistanceTimer.seconds() >= TIME_FRONT_DIST_RESET
        ) {
            // read i2c
            color1.update();
            color2.update();
            hsv1 = color1.getHSV();
            hsv2 = color2.getHSV();
            a1 = Artifact.fromHSV(hsv1);
            a2 = Artifact.fromHSV(hsv2);

            artifacts[currentFrontSlot] = (a1 == GREEN || a2 == GREEN) ? GREEN : PURPLE;
            frontDistanceTimer.reset();

            // combine Artifact reading from both color sensors
//            artifacts[currentFrontSlot] = a1.or(a2);
            updateLEDs();

//            if (artifacts[currentFrontSlot] != EMPTY)
                genFeedingOrder.run();
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
        int slotToFront = slotGoingToFront();
        if (
                intakePower >= 0 &&
                intakePower < INTAKE_POWER_OMNI_CONTACT &&
                (
                        omniSlot != -1 && artifacts[omniSlot] != EMPTY ||
                        slotToFront != -1 && artifacts[slotToFront] != EMPTY
                ) // artifact touching omni wheel
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

    private double lastRadians = getTargetRadians(0, Zone.INTAKE_SENSORS);
    private final ElapsedTime wrapAroundTimer = new ElapsedTime();
    private boolean wrapAround = true;

    private double realTarget;

    /**
     * @param slot Slot you wish to move (0, 1 or 2)
     */
    void moveSlot(int slot, Zone target) {
        realTarget = target.radians - 2 * PI / 3 * slot;

        double newRadians = getTargetRadians(slot, target);
        if (newRadians == lastRadians)
            return;

        double front0 = getTargetRadians(0, Zone.INTAKE_SENSORS);
        double front1 = getTargetRadians(1, Zone.INTAKE_SENSORS);

        wrapAround = lastRadians == front0 && newRadians == front1 ||
                    lastRadians == front1 && newRadians == front0;

        lastRadians = newRadians;

        if (wrapAround) {
            servo.turnToAngle(-PI);
            wrapAroundTimer.reset();
        } else
            servo.turnToAngle(lastRadians);
    }

    private static double getTargetRadians(int slot, Zone target) {
        return normalizeRadians(ROTOR_OUTPUT_OFFSET + (
                slot == 0 ? target.radians == 0 ? 0 : OFFSET_0_BACK :
                slot == 1 ? target.radians == 0 ? OFFSET_1_FRONT : OFFSET_1_BACK :
                            target.radians == 0 ? OFFSET_2_FRONT : OFFSET_2_BACK
        ));
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

    int slotGoingToFront() {
        double slotID = normalizeRadians(Zone.INTAKE_SENSORS.radians - realTarget) / (2 * PI / 3);
        for (int i = 0; i < artifacts.length; i++)
            if (abs(i - slotID) <= 0.1)
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
        telemetry.addData("Slot 0 position (deg)", toDegrees(position));
        telemetry.addData("Error (deg)", toDegrees(abs(normalizeRadians(lastRadians - position))));
        telemetry.addLine();
        telemetry.addData("Artifact 1", a1);
        hsv1.printTo(telemetry);
        telemetry.addLine();
        telemetry.addData("Artifact 2", a2);
        hsv2.printTo(telemetry);
        telemetry.addLine();
        telemetry.addData("Front dist (mm)", front1.getReading());
        telemetry.addData("Back dist (mm)", back1.getReading());
    }

}
