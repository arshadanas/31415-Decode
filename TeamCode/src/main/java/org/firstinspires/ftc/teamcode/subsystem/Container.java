package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.PURPLE;
import static org.firstinspires.ftc.teamcode.subsystem.Container.SlotTarget.BACK;
import static org.firstinspires.ftc.teamcode.subsystem.Container.SlotTarget.FRONT;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

import java.util.Arrays;

@Config
public final class Container {

    public static HSV
            minPurple = new HSV(
                    175,
                    0.4,
                    0
            ),
            maxPurple = new HSV(
                    350,
                    1,
                    1
            ),

            minGreen = new HSV(
                    60,
                    0.65,
                    0
            ),
            maxGreen = new HSV(
                    160,
                    1,
                    1
            );

    public static Artifact hsvToArtifact(HSV hsv) {
        return
                hsv.between(minPurple, maxPurple) ? PURPLE :
                hsv.between(minGreen, maxGreen) ?   GREEN :
                                                    EMPTY;
    }

    public static PIDGains pidGains = new PIDGains(0, 0, 0);

    public static double
            ABS_OFFSET_ROTOR = 0,
            TOLERANCE_FRONT_RADIANS = toRadians(5),
            TOLERANCE_BACK_RADIANS = toRadians(5),
            THRESHOLD_FRONT_MM = 40,
            THRESHOLD_BACK_MM = 80,
            INTAKE_SPEED_WHEN_SORTING = 0.5,
            ROTOR_SPEED_THRESHOLD_INTAKE_SPIN = 0.5;

    // hardware
    private final CRServo[] servos;
    private final AnalogSensor encoder, frontDist1, backDist1;
    private final ColorSensor color1, color2;
    private final LEDIndicator[] indicators;

    /**
     * Position of slot 0, in radians
     */
    private double position = 0;

    /**
     * Position of given slot, in radians
     */
    private double getPositionOf(int slot) {
        return position + slot * 2 * PI / 3.0;
    }

    private final Artifact[] slots = {EMPTY, EMPTY, EMPTY};
    private final Artifact[] sorted = slots.clone();

    private final PIDController controller = new PIDController();

    private int selectedSlot = 0;

    private SlotTarget target = FRONT;

    enum SlotTarget {
        FRONT(0),
        BACK(PI);

        private final double radians;
        SlotTarget(double radians) {
            this.radians = radians;
        }

        private double getTolerance() {
            return this == SlotTarget.BACK ? TOLERANCE_BACK_RADIANS : TOLERANCE_FRONT_RADIANS;
        }
    }

    Container(HardwareMap hardwareMap) {
        servos = new CRServo[]{
                hardwareMap.get(CRServo.class, "rotor 1"),
                hardwareMap.get(CRServo.class, "rotor 2")
        };

        encoder = new AnalogSensor(hardwareMap, "rotor", 3 * 2 * PI);

        frontDist1 = new AnalogSensor(hardwareMap, "front 1", 1300);
        backDist1 = new AnalogSensor(hardwareMap, "back 1", 1000);

        color1 = new ColorSensor(hardwareMap, "color 1", 1);
        color2 = new ColorSensor(hardwareMap, "color 2", 1);

        indicators = new LEDIndicator[]{
                new LEDIndicator(hardwareMap, "led 1a", "led 1b"),
                new LEDIndicator(hardwareMap, "led 2a", "led 2b"),
                new LEDIndicator(hardwareMap, "led 3a", "led 3b")
        };
    }

    void run() {
        position = normalizeRadians(encoder.getReading() + ABS_OFFSET_ROTOR);

        // check front slot sensors
        for (int i = 0; i < slots.length; i++)
            if ( // TODO check rotor speed under threshold
                    atPosition(i, FRONT) &&
                    slots[i] == EMPTY &&
                    frontDist1.getReading() < THRESHOLD_FRONT_MM
            ) {
                color1.update();
                color2.update();
                set(i, hsvToArtifact(color1.getHSV()). or (hsvToArtifact(color2.getHSV())));
                break;
            }

        // check back slot sensors
        for (int i = 0; i < slots.length; i++)
            if ( // TODO check rotor speed under threshold
                    atPosition(i, BACK) &&
                    slots[i] != EMPTY &&
                    backDist1.getReading() > THRESHOLD_BACK_MM
            ) {
                set(i, EMPTY);
                break;
            }

        // LEDs
        int n = 0;
        for (Artifact a : slots)
            if (a != EMPTY)
                sorted[n++] = a;

        while (n < sorted.length)
            sorted[n++] = EMPTY;

        for (int i = 0; i < sorted.length; i++)
            indicators[i].setColor(sorted[i].toLEDColor());

        // PID
        controller.setGains(pidGains);
        controller.setTarget(new State(getPositionOf(selectedSlot) + getError(selectedSlot, target)));
        double power = controller.calculate(new State(getPositionOf(selectedSlot)));
        rotorAboveThreshold = power > ROTOR_SPEED_THRESHOLD_INTAKE_SPIN;
        for (CRServo servo : servos)
            servo.setPower(power);
    }

    private boolean rotorAboveThreshold = false;

    /**
     * When we spin the rotor, if an {@link Artifact} is in the front (TBA), the intake omni wheel must spin to contain the {@link Artifact}
     */
    double getMinIntakeSpeed() {
        return rotorAboveThreshold ? INTAKE_SPEED_WHEN_SORTING : 0;
    }

    int getNearestFeedSlot() {
        double min = Double.MAX_VALUE;
        int minInd = -1;
        for (int i = 0; i < 3; i++) {
            double error = getError(i, BACK);
            if (error < min){
                min = error;
                minInd = i;
            }
        }
        return minInd;
    }

    int getNearestFeedSlot(Artifact color) {
        double min = Double.MAX_VALUE;
        int minInd = -1;
        for (int i = 0; i < 3; i++) {
            if (slots[i] != color)
                continue;

            double error = getError(i, BACK);
            if (error < min){
                min = error;
                minInd = i;
            }
        }
        return minInd;
    }

    /**
     * Add an {@link Artifact} to the slot currently in the intaking position <br>
     * Then move the next empty slot to the front for intaking (or nearest feed if no empty slots)
     */
    private void set(int slot, Artifact color) {

        slots[slot] = color;

        if (slots[slot] == EMPTY)
            return;

        int nextEmptySlot = EMPTY.firstOccurrenceIn(slots);
        boolean hasEmptySlot = nextEmptySlot != -1;

        moveSlot(hasEmptySlot ? nextEmptySlot : getNearestFeedSlot(), hasEmptySlot ? FRONT : BACK);
    }

    boolean isFull() {
        return EMPTY.numOccurrencesIn(slots) == 0;
    }

    Artifact[] getSlots() {
        return slots.clone();
    }

    /**
     * @return Artifact in specified slot
     */
    Artifact getArtifact(int slot) {
        return slots[slot];
    }

    /**
     * @param slot      Slot you wish to move (0, 1 or 2)
     * @param target  True for intaking position, false for feeding/shooting position
     */
    void moveSlot(int slot, SlotTarget target) {
        this.selectedSlot = slot;
        this.target = target;
    }

    /**
     * @return  If the given slot is at the given target, within tolerance in either direction
     */
    private boolean atPosition(int slot, SlotTarget target) {
        return abs(getError(slot, target)) <= target.getTolerance();
    }

    /**
     * @return  Distance, in radians, between given slot's position and given target
     */
    private double getError(int slot, SlotTarget target) {
        return normalizeRadians(target.radians - getPositionOf(slot));
    }

    void print(Telemetry telemetry) {
        telemetry.addData("SORTER", Arrays.toString(slots));
        telemetry.addLine();
        telemetry.addData("Slot 0 position (rad)", getPositionOf(0));
        telemetry.addData("Slot 1 position (rad)", getPositionOf(1));
        telemetry.addData("Slot 2 position (rad)", getPositionOf(2));
        telemetry.addLine();
        telemetry.addData("Slot 0 position (deg)", toDegrees(getPositionOf(0)));
        telemetry.addData("Slot 1 position (deg)", toDegrees(getPositionOf(1)));
        telemetry.addData("Slot 2 position (deg)", toDegrees(getPositionOf(2)));
        telemetry.addLine();
        telemetry.addData("Front distance (mm)", frontDist1.getReading());
        telemetry.addData("Back distance (mm)", backDist1.getReading());
    }

}
