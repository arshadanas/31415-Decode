package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;

@Config
public final class Container {

    public static PIDGains pidGains = new PIDGains(0, 0, 0);

    public static double
            ABS_OFFSET_ROTOR = 0,
            TOLERANCE_RADIANS = toRadians(5);

    private final CRServo[] servos;

    private final AnalogSensor encoder;

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

    private final LEDIndicator[] indicators;

    private final PIDController controller = new PIDController();

    private int selectedSlot = 0;
    private boolean isIntaking = false;

    Container(HardwareMap hardwareMap) {
        servos = new CRServo[]{
                hardwareMap.get(CRServo.class, "rotor 1"),
                hardwareMap.get(CRServo.class, "rotor 2")
        };

        encoder = new AnalogSensor(hardwareMap, "rotor", 3 * 2 * PI);

        indicators = new LEDIndicator[]{
                new LEDIndicator(hardwareMap, "led 1a", "led 1b"),
                new LEDIndicator(hardwareMap, "led 2a", "led 2b"),
                new LEDIndicator(hardwareMap, "led 3a", "led 3b")
        };
    }

    void run() {
        position = normalizeRadians(encoder.getReading() + ABS_OFFSET_ROTOR);

        controller.setGains(pidGains);

        controller.setTarget(new State(getPositionOf(selectedSlot) + getError(selectedSlot, isIntaking)));

        double power = controller.calculate(new State(getPositionOf(selectedSlot)));
        for (CRServo servo : servos)
            servo.setPower(power);

        for (int i = 0; i < slots.length; i++)
            indicators[i].setColor(slots[i].toLEDColor());
    }

    int getNearestFeedSlot() {
        double min = Double.MAX_VALUE;
        int minInd = -1;
        for (int i = 0; i < 3; i++) {
            double error = getError(i, false);
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

            double error = getError(i, false);
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
     * @return Whether the {@link Artifact} was successfully added to the {@link Container}
     */
    boolean add(Artifact artifact) {

        boolean added = false;
        for (int i = 0; i < slots.length; i++)
            if (slots[i] == EMPTY && atPosition(i, true)) {
                slots[i] = artifact;
                added = true;
                break;
            }
        if (!added)
            return false;

        int nextEmptySlot = EMPTY.firstOccurrenceIn(slots);
        boolean hasEmptySlot = nextEmptySlot != -1;

        moveSlot(hasEmptySlot ? nextEmptySlot : getNearestFeedSlot(), hasEmptySlot);
        return true;
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
     * @param intaking  True for intaking position, false for feeding/shooting position
     */
    void moveSlot(int slot, boolean intaking) {
        this.selectedSlot = slot;
        this.isIntaking = intaking;
    }

    /**
     * @param isIntaking  True for intaking position, false for feeding/shooting position
     * @return  If the given slot is at the given target, within {@link #TOLERANCE_RADIANS} in either direction
     */
    private boolean atPosition(int slot, boolean isIntaking) {
        return abs(getError(slot, isIntaking)) <= TOLERANCE_RADIANS;
    }

    /**
     * @param isIntaking  True for intaking position (0 rad), false for feeding/shooting position (PI rad)
     * @return  Distance, in radians, between given slot's position and given target
     */
    private double getError(int slot, boolean isIntaking) {
        return normalizeRadians((isIntaking ? 0 : PI) - getPositionOf(slot));
    }

    void print(Telemetry telemetry) {
        telemetry.addLine("SORTER:");
        telemetry.addLine();
        telemetry.addData("Slot 0", slots[0]);
        telemetry.addData("Slot 1", slots[1]);
        telemetry.addData("Slot 2", slots[2]);
        telemetry.addLine();
        telemetry.addData("Slot 0 position (rad)", getPositionOf(0));
        telemetry.addData("Slot 1 position (rad)", getPositionOf(1));
        telemetry.addData("Slot 2 position (rad)", getPositionOf(2));
        telemetry.addLine();
        telemetry.addData("Slot 0 position (deg)", toDegrees(getPositionOf(0)));
        telemetry.addData("Slot 1 position (deg)", toDegrees(getPositionOf(1)));
        telemetry.addData("Slot 2 position (deg)", toDegrees(getPositionOf(2)));
    }

}
