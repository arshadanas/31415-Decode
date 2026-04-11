package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.Ranges;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;

@Config
public final class Rotor {

    public static double
            ENCODER_OFFSET = -1.5742869852988852,
            OFFSET_0_FRONT = -0.12571301470111473,
            OFFSET_0_BACK = 0.18269433170909233,
            OFFSET_1_FRONT = 0.2686820876920806,
            OFFSET_1_BACK = 0.0020894341022874574,
            OFFSET_2_FRONT = 0.12989188290568965,
            OFFSET_2_BACK = -0.20351546350451732,

            TIME_WRAPAROUND = 0.03,

            TOLERANCE_INTAKE_SENSOR = 0.2, // too high => false positives, too low => false negatives (no-detect)
            TOLERANCE_FEEDER = 0.15, // too high => false negatives (removals)
            TOLERANCE_INTAKE_OMNI = 0.5236;

    private final CachedSimpleServo servo;
    private final AnalogSensor encoder;

    private double lastServoTarget;
    private final ElapsedTime wrapAroundTimer = new ElapsedTime();
    private boolean wrapAround = false;

    double slot0Position, slot0Target;

    private static double offsetRadians(double slot0Radians, int numSlotsCCW) {
        return normalizeRadians(slot0Radians + numSlotsCCW * 2 * PI / 3.0);
    }

    Rotor(HardwareMap hardwareMap) {
        servo = new CachedSimpleServo(hardwareMap, "rotor 2", -PI, PI);
        encoder = new AnalogSensor(hardwareMap, "rotor", 2 * PI);

        // slot nearest to feeder because we preload with a slot aligned to the feeder
        slot0Position = normalizeRadians(encoder.getReading() + ENCODER_OFFSET);
        int nearestFeedSlot = Zone.FEEDER.getNearestSlot(slot0Position, Handler.FULL, true);
        slot0Target = offsetRadians(Zone.FEEDER.radians, -nearestFeedSlot);
        lastServoTarget = Zone.INTAKE_SENSOR.getServoTarget(nearestFeedSlot);
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

        double front0 = Zone.INTAKE_SENSOR.getServoTarget(0);
        double front1 = Zone.INTAKE_SENSOR.getServoTarget(1);

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

    public enum Zone {
        INTAKE_SENSOR(0),
        INTAKE_OMNI(0),
        FEEDER(PI);

        private final double radians;
        Zone(double radians) {
            this.radians = radians;
        }

        private double getTolerance() {
            switch (this) {
                case INTAKE_SENSOR: return TOLERANCE_INTAKE_SENSOR;
                case INTAKE_OMNI:   return TOLERANCE_INTAKE_OMNI;
                case FEEDER:        return TOLERANCE_FEEDER;
            }
            return 0;
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

        boolean slotIsHere(double slot0Reference, int slot) {
            return abs(distFrom(slot0Reference, slot)) <= this.getTolerance();
        }

        /**
         * @return Filled slot in this {@link Zone}
         */
        int getFilledSlotHere(double slot0Reference, boolean[] isFilled) {
            return
                    isFilled[0] && slotIsHere(slot0Reference, 0) ? 0 :
                    isFilled[1] && slotIsHere(slot0Reference, 1) ? 1 :
                    isFilled[2] && slotIsHere(slot0Reference, 2) ? 2 :
                                                                        -1;
        }

        /**
         * @param getFilled true to get filled slots, false to get empty slots
         * @return Slot closest to this {@link Zone} that is either filled or empty (per getFilled)
         */
        int getNearestSlot(double slot0Reference, boolean[] isFilled, boolean getFilled) {
            double min = Double.MAX_VALUE;
            int minInd = -1;
            for (int i = 0; i < 3; i++) if (isFilled[i] == getFilled) {
                double error = abs(distFrom(slot0Reference, i));
                if (error < min) {
                    min = error;
                    minInd = i;
                }

            }
            return minInd;
        }
    }
}
