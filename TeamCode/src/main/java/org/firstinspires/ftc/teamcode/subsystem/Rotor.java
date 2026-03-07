package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
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

    public static void main(String... args) {
        System.out.println(Zone.INTAKE_SENSORS.getServoTarget(0));
        System.out.println(Zone.FEEDER_SENSORS.getServoTarget(0));
        System.out.println(Zone.INTAKE_SENSORS.getServoTarget(1));
        System.out.println(Zone.FEEDER_SENSORS.getServoTarget(1));
        System.out.println(Zone.INTAKE_SENSORS.getServoTarget(2));
        System.out.println(Zone.FEEDER_SENSORS.getServoTarget(2));
    }

    public static double
            ENCODER_OFFSET = -1.5742869852988852,
            OFFSET_0_FRONT = -0.12571301470111473,
            OFFSET_0_BACK = 0.18269433170909233,
            OFFSET_1_FRONT = 0.2686820876920806,
            OFFSET_1_BACK = 0.0020894341022874574,
            OFFSET_2_FRONT = 0.12989188290568965,
            OFFSET_2_BACK = -0.20351546350451732,

            TIME_WRAPAROUND = 0.03,

            TOLERANCE_INTAKE_SENSORS_DEG = 11.46, // too high => false positives, too low => false negatives (no-detect)
            TOLERANCE_FEEDER_SENSORS_DEG = 8.6, // too high => false negatives (removals)
            TOLERANCE_FEEDER_OMNIS_DEG = 30,
            TOLERANCE_INTAKE_OMNI_DEG = 30;

    private final CachedSimpleServo servo;
    private final AnalogSensor encoder;

    private double lastServoTarget;
    private final ElapsedTime wrapAroundTimer = new ElapsedTime();
    private boolean wrapAround = false;

    double slot0Position, slot0Target;

    private static double offsetRadians(double slot0Radians, int numSlotsCCW) {
        return normalizeRadians(slot0Radians + numSlotsCCW * 2 * PI / 3.0);
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

        private double getServoTarget(int slot) {
            slot = Ranges.wrap(slot, 0, 3);
            return normalizeRadians(offsetRadians(radians, -slot) + ENCODER_OFFSET + (
                    slot == 0 ? radians == 0 ? OFFSET_0_FRONT : OFFSET_0_BACK :
                    slot == 1 ? radians == 0 ? OFFSET_1_FRONT : OFFSET_1_BACK :
                                radians == 0 ? OFFSET_2_FRONT : OFFSET_2_BACK
            ));
        }

        /**
         * @return Distance, in radians, between given slot and this zone's specific {@link #radians}
         */
        double distFrom(double slot0Reference, int slot) {
            return normalizeRadians(this.radians - offsetRadians(slot0Reference, slot));
        }

        /**
         * @return Slot in this zone that satisfies the predicate
         */
        int getSlotHere(double slot0Reference, IntPredicate predicate) {
            for (int i = 0; i < 3; i++)
                if (predicate.test(i) && abs(distFrom(slot0Reference, i)) <= this.getTolerance())
                    return i;
            return -1;
        }

        /**
         * @return Slot closest to this zone that satisfies the predicate
         */
        int getNearestSlot(double slot0Reference, IntPredicate predicate) {
            double min = Double.MAX_VALUE;
            int minInd = -1;
            for (int i = 0; i < 3; i++)
                if (predicate.test(i)) {
                    double error = abs(distFrom(slot0Reference, i));
                    if (error < min) {
                        min = error;
                        minInd = i;
                    }
                }
            return minInd;
        }
    }

    Rotor(HardwareMap hardwareMap) {
        servo = new CachedSimpleServo(hardwareMap, "rotor 2", -PI, PI);
        encoder = new AnalogSensor(hardwareMap, "rotor", 2 * PI);

        // slot nearest to feeder because we preload with a slot aligned to the feeder
        int nearestFeedSlot = Zone.FEEDER_SENSORS.getNearestSlot(
                slot0Target = slot0Position = normalizeRadians(encoder.getReading() + ENCODER_OFFSET),
                i -> true
        );
        if (nearestFeedSlot != -1)
            this.lastServoTarget = Zone.INTAKE_SENSORS.getServoTarget(nearestFeedSlot);
    }

    void run() {

        slot0Position = normalizeRadians(encoder.getReading() + ENCODER_OFFSET);

        if (wrapAround && wrapAroundTimer.seconds() >= TIME_WRAPAROUND) {
            wrapAround = false;
            servo.turnToAngle(lastServoTarget);
        }
    }

    /**
     * @param slot Slot you wish to move (0, 1 or 2)
     */
    public void moveSlot(int slot, Zone target) {
        this.slot0Target = offsetRadians(target.radians, -slot);

        double newServoTarget = target.getServoTarget(slot);
        if (newServoTarget == lastServoTarget)
            return;

        double front0 = Zone.INTAKE_SENSORS.getServoTarget(0);
        double front1 = Zone.INTAKE_SENSORS.getServoTarget(1);

        wrapAround = lastServoTarget == front0 && newServoTarget == front1 ||
                lastServoTarget == front1 && newServoTarget == front0;

        lastServoTarget = newServoTarget;

        if (wrapAround) {
            servo.turnToAngle(-PI);
            wrapAroundTimer.reset();
        } else
            servo.turnToAngle(lastServoTarget);
    }

    void printTo(Telemetry telemetry) {
        telemetry.addLine("ROTOR:");
        telemetry.addLine();
        telemetry.addData("Slot 0 position (deg)", toDegrees(slot0Position));
        telemetry.addData("Slot 0 target (deg)", toDegrees(slot0Target));
        telemetry.addData("Error (deg)", toDegrees(normalizeRadians(slot0Target - slot0Position)));
    }
}
