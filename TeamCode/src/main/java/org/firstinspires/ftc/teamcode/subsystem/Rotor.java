package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.control.Ranges.wrap;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.Ranges;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;

import java.util.function.IntPredicate;

@Config
public final class Rotor {

    public static double
            ROTOR_ENCODER_OFFSET = -1.5742869852988852,
            ROTOR_OUTPUT_OFFSET = -1.7,
            OFFSET_0_BACK = 3.45,
            OFFSET_1_FRONT = -1.7,
            OFFSET_1_BACK = 1.175,
            OFFSET_2_FRONT = 2.35,
            OFFSET_2_BACK = -1.125,

            TIME_WRAPAROUND = 0.03,

            TOLERANCE_INTAKE_SENSORS_DEG = 11.46, // too high => false positives, too low => false negatives (no-detect)
            TOLERANCE_FEEDER_SENSORS_DEG = 8.6, // too high => false negatives (removals)
            TOLERANCE_FEEDER_OMNIS_DEG = 30,
            TOLERANCE_INTAKE_OMNI_DEG = 30;

    private final CachedSimpleServo servo;
    private final AnalogSensor encoder;

    private double lastServoTarget = getServoTarget(0, Zone.INTAKE_SENSORS);
    private final ElapsedTime wrapAroundTimer = new ElapsedTime();
    private boolean wrapAround = true;

    /**
     * Position of slot 0, in radians
     */
    private double position = 0, target = 0;

    private static double mapRadians(double slot0Radians, int slot) {
        return normalizeRadians(slot0Radians + wrap(slot, 0, 3) * 2 * PI / 3.0);
    }

    public enum Zone {
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

    Rotor(HardwareMap hardwareMap) {
        servo = new CachedSimpleServo(hardwareMap, "rotor 2", -PI, PI);
        encoder = new AnalogSensor(hardwareMap, "rotor", 2 * PI);
    }

    void run() {

        position = normalizeRadians(encoder.getReading() + ROTOR_ENCODER_OFFSET);

        if (wrapAround && wrapAroundTimer.seconds() >= TIME_WRAPAROUND) {
            wrapAround = false;
            servo.turnToAngle(lastServoTarget);
        }
    }

    /**
     * @param slot Slot you wish to move (0, 1 or 2)
     */
    public void moveSlot(int slot, Zone target) {
        slot = Ranges.wrap(slot, 0, 3);
        this.target = normalizeRadians(target.radians - 2 * PI / 3 * slot);

        double newServoTarget = getServoTarget(slot, target);
        if (newServoTarget == lastServoTarget)
            return;

        double front0 = getServoTarget(0, Zone.INTAKE_SENSORS);
        double front1 = getServoTarget(1, Zone.INTAKE_SENSORS);

        wrapAround = lastServoTarget == front0 && newServoTarget == front1 ||
                lastServoTarget == front1 && newServoTarget == front0;

        lastServoTarget = newServoTarget;

        if (wrapAround) {
            servo.turnToAngle(-PI);
            wrapAroundTimer.reset();
        } else
            servo.turnToAngle(lastServoTarget);
    }

    private static double getServoTarget(int slot, Zone target) {
        return normalizeRadians(ROTOR_OUTPUT_OFFSET + (
                slot == 0 ? target.radians == 0 ? 0 : OFFSET_0_BACK :
                slot == 1 ? target.radians == 0 ? OFFSET_1_FRONT : OFFSET_1_BACK :
                            target.radians == 0 ? OFFSET_2_FRONT : OFFSET_2_BACK
        ));
    }

    /**
     * @return The (index of the) slot currently at the given target, -1 if no slot at that position
     */
    int getSlotAt(Zone target) {
        for (int i = 0; i < 3; i++)
            if (atPosition(i, target))
                return i;
        return -1;
    }

    int slotGoingToFront() {
        return wrap((int) -Math.round(target / (2 * PI / 3)), 0, 3);
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
        return normalizeRadians(target.radians - mapRadians(position, slot));
    }

    /**
     * @return Slot closest to the specified zone that satisfies the predicate. -1 if no such slot found
     */
    int getNearestSlot(IntPredicate predicate, Rotor.Zone zone) {
        double min = Double.MAX_VALUE;
        int minInd = -1;
        for (int i = 0; i < 3; i++) if (predicate.test(i)) {
            double error = abs(getError(i, zone));
            if (error < min) {
                min = error;
                minInd = i;
            }
        }
        return minInd;
    }

    void printTo(Telemetry telemetry) {
        telemetry.addLine("ROTOR:");
        telemetry.addLine();
        telemetry.addData("Slot 0 position (deg)", toDegrees(position));
        telemetry.addData("Slot 0 target (deg)", toDegrees(target));
        telemetry.addData("Error (deg)", toDegrees(normalizeRadians(target - position)));
    }
}
