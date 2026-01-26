package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystem.Container.SlotTarget.BACK;
import static org.firstinspires.ftc.teamcode.subsystem.Container.SlotTarget.FRONT;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

import java.util.Arrays;

@Config
public final class Container {

    public static PIDGains pidGains = new PIDGains(0, 0, 0);

    public static double
            ABS_OFFSET_ROTOR = -1.2966209679361516,
            TOLERANCE_FRONT_RADIANS = toRadians(5),
            TOLERANCE_BACK_RADIANS = toRadians(5),
            THRESHOLD_FRONT_MM = 70, // start of ramp = ~115
            THRESHOLD_BACK_MM = 70, // above rotor = ~75
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

        intake = new CachedMotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intake.setInverted(true);
        intake.setZeroPowerBehavior(FLOAT);
    }

    private final CachedMotorEx intake;

    void run() {
        position = normalizeRadians(encoder.getReading() + ABS_OFFSET_ROTOR);

        int frontSlot = getSlotAt(FRONT);
        if (
                frontSlot != -1 &&
                slots[frontSlot] == EMPTY &&
                frontDist1.getReading() < THRESHOLD_FRONT_MM
                // TODO check rotor speed under threshold
        ) {
            color1.update();
            color2.update();
            // combine Artifact reading from both color sensors
            slots[frontSlot] = Artifact.fromHSV(color1.getHSV()). or (Artifact.fromHSV(color2.getHSV()));

            if (slots[frontSlot] != EMPTY) {
                int nextEmptySlot = EMPTY.firstOccurrenceIn(slots);

                if (nextEmptySlot == -1) // no empty slots
                    moveSlot(getNearestFeedSlot(), BACK); // move artifact to feeder
                else
                    moveSlot(nextEmptySlot, FRONT);
            }
        }

        // check back slot sensors
        int backSlot = getSlotAt(BACK);
        if (
                backSlot != -1 &&
                slots[backSlot] != EMPTY &&
                backDist1.getReading() > THRESHOLD_BACK_MM
                // TODO check rotor speed under threshold
        )
            slots[backSlot] = EMPTY;

        // LEDs
        int n = 0;
        for (Artifact a : slots)
            if (a != EMPTY)
                indicators[n++].setColor(a.toLEDColor());
        while (n < slots.length)
            indicators[n++].setColor(EMPTY.toLEDColor());

        // PID
        controller.setGains(pidGains);
        controller.setTarget(new State(getError(selectedSlot, target)));
        double power = controller.calculate(new State());
        rotorAboveThreshold = power > ROTOR_SPEED_THRESHOLD_INTAKE_SPIN;
        for (CRServo servo : servos)
            servo.setPower(power);

        intake.set(getMinIntakeSpeed());
    }

    /** //TODO only spin intake if ball near the front
     * When we spin the rotor, if an {@link Artifact} is in the front (TBA), the intake omni wheel must spin to contain the {@link Artifact}
     */
    double getMinIntakeSpeed() {
        return rotorAboveThreshold ? INTAKE_SPEED_WHEN_SORTING : 0;
    }

    void print(Telemetry telemetry) {
        telemetry.addData("CONTAINER", Arrays.toString(slots));
        telemetry.addLine();
        telemetry.addData("Current (deg)", toDegrees(getPositionOf(selectedSlot)));
        telemetry.addData("Target (deg", toDegrees(target.radians));
        telemetry.addData("Error (deg)", toDegrees(getError(selectedSlot, target)));
        telemetry.addData("Error deriv (deg/s)", toDegrees(controller.getFilteredErrorDerivative()));
        telemetry.addLine();
        telemetry.addData("Slot 0 pos (rad)", position);
        telemetry.addData("Slot 0 pos (deg)", toDegrees(position));
        telemetry.addLine();
        telemetry.addData("Front dist (mm)", frontDist1.getReading());
        telemetry.addData("Back dist (mm)", backDist1.getReading());
    }


    private boolean rotorAboveThreshold = false;

    private int getNearestFeedSlot() {
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

    private int getNearestFeedSlot(Artifact color) {
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
     * @return The (index of the) slot currently at the given target, -1 if no slot at that position
     */
    private int getSlotAt(SlotTarget target) {
        for (int i = 0; i < slots.length; i++)
            if (atPosition(i, target))
                return i;
        return -1;
    }

    /**
     * @param slot Slot you wish to move (0, 1 or 2)
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

}
