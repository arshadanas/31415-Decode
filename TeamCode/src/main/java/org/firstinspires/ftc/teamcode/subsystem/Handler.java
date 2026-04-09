package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.control.Ranges.wrap;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.PURPLE;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedDcMotor;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.IntPredicate;

@Config
public final class Handler {

    public static double
            ANGLE_PRESSER_RETRACTED = 87,
            ANGLE_PRESSER_EXTENDED = 211,
            ANGLE_PRESSER_L_OFFSET = -37,

            SPEED_IDLE_FEEDER = 0,
            THRESHOLD_FRONT_MM = 65, // start of ramp = ~115
            INTAKE_POWER_OMNI_CONTACT = 0.4,

            TIME_FRONT_DIST_COOLDOWN = 0.1,
            TIME_FEED = 1,
            TIME_FEED_LAST_EXTRA = 0.4,

            CACHE_THRESHOLD_INTAKE = 0.05,
            CACHE_THRESHOLD_FEEDER = 0.05;

    private final Rotor rotor;
    private final CachedMotorEx intake;
    private final CachedDcMotor[] feeder;
    private final AnalogSensor front1;

    private double intakePower;
    public void setIntake(double power) {
        this.intakePower = power;
    }

    private double manualFeederPower;
    public void setFeederManual(double power) {
        this.manualFeederPower = power;
    }

    private int lastSlotMoved;
    public void moveRotor() {
        rotor.moveSlot(lastSlotMoved++, Rotor.Zone.FEEDER_SENSORS);
    }

    private final ArrayList<Integer> feedingOrder = new ArrayList<>();
    private final ElapsedTime timeSinceIntaked = new ElapsedTime(), timeSpentFeeding = new ElapsedTime();

    public final Artifact[] artifacts = {EMPTY, EMPTY, EMPTY};
    private final IntPredicate
            emptySlot = i -> artifacts[i] == EMPTY,
            filledSlot = i -> artifacts[i] != EMPTY,
            anySlot = i -> true;

    public boolean hasArtifacts() {
        return artifacts[0] != EMPTY || artifacts[1] != EMPTY || artifacts[2] != EMPTY;
    }
    public boolean isFull() {
        return artifacts[0] != EMPTY && artifacts[1] != EMPTY && artifacts[2] != EMPTY;
    }

    public void setContents(Artifact[] artifacts) {
        assert artifacts.length == 3;
        System.arraycopy(artifacts, 0, this.artifacts, 0, 3);
        feedFastest();
    }

    Handler(HardwareMap hardwareMap) {

        rotor = new Rotor(hardwareMap);

        front1 = new AnalogSensor(hardwareMap, "front 1", 4000);

        intake = new CachedMotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intake.setInverted(true);
        intake.setZeroPowerBehavior(FLOAT);

        feeder = new CachedDcMotor[]{
                new CachedDcMotor(hardwareMap.get(CRServo.class, "feeder R")),
                new CachedDcMotor(hardwareMap.get(CRServo.class, "feeder L"))
        };
        feeder[0].motor.setDirection(REVERSE);
    }

    void run(boolean feed) {

        int nearestEmptySlot = Rotor.Zone.INTAKE_SENSORS.getNearestSlot(rotor.slot0Position, emptySlot);

        if (intakePower > 0 && nearestEmptySlot != -1) // move empty slot to intake
            rotor.moveSlot(nearestEmptySlot, Rotor.Zone.INTAKE_SENSORS);
        else if (!feedingOrder.isEmpty())
            rotor.moveSlot(feedingOrder.get(0), Rotor.Zone.FEEDER_SENSORS);

        if (
                nearestEmptySlot != -1 && // there is an empty slot
                Rotor.Zone.INTAKE_SENSORS.slotIsHere(rotor.slot0Position, nearestEmptySlot) && // it is at the front
                front1.getReading() < THRESHOLD_FRONT_MM && // there is something in front of the distance sensor
                timeSinceIntaked.seconds() >= TIME_FRONT_DIST_COOLDOWN // long enough for distance sensor to refresh
        ) {
            artifacts[nearestEmptySlot] = PURPLE;
            timeSinceIntaked.reset();
            feedFastest();
        }

        int backSlot = Rotor.Zone.FEEDER_SENSORS.getSlotHere(rotor.slot0Position, anySlot);

        boolean backSlotIsFeedTarget = !feedingOrder.isEmpty() && backSlot == feedingOrder.get(0);

        if (backSlot == -1 || !backSlotIsFeedTarget)
            timeSpentFeeding.reset();
        else {
            boolean lastArtifact = artifacts[(backSlot + 1) % 3] == EMPTY && artifacts[(backSlot + 2) % 3] == EMPTY;
            if (timeSpentFeeding.seconds() >= TIME_FEED + (lastArtifact ? TIME_FEED_LAST_EXTRA : 0)) {
                artifacts[backSlot] = EMPTY;
                feedingOrder.remove(0);
            }
        }

        double feederPower =
                manualFeederPower != 0 ? manualFeederPower :
                feed && (backSlot == -1 || artifacts[backSlot] == EMPTY || backSlotIsFeedTarget) ? 1 : SPEED_IDLE_FEEDER;

        for (CachedDcMotor servo : feeder) {
            servo.threshold = CACHE_THRESHOLD_FEEDER;
            servo.setPower(feederPower);
        }

        intake.threshold = CACHE_THRESHOLD_INTAKE;
        intake.set(adaptiveClipIntakePower(intakePower));

        rotor.run();
    }

    /**
     * Requires minimum intake power of {@link #INTAKE_POWER_OMNI_CONTACT} when there is an
     * {@link Artifact} touching the front omni wheel
     */
    private double adaptiveClipIntakePower(double intakePower) {
        int filledOmniSlot = Rotor.Zone.INTAKE_OMNI.getSlotHere(rotor.slot0Position, filledSlot);
        int filledSlotMovingToOmni = Rotor.Zone.INTAKE_OMNI.getSlotHere(rotor.slot0Target, filledSlot);

        return (filledOmniSlot != -1 || filledSlotMovingToOmni != -1) && intakePower >= 0 && intakePower < INTAKE_POWER_OMNI_CONTACT ? INTAKE_POWER_OMNI_CONTACT :
                intakePower;
    }

    boolean feedsPending() {
        return !feedingOrder.isEmpty();
    }

    /**
     * Generate the most efficient feeding order
     */
    public void feedFastest() {
        feedingOrder.clear();

        int first = Rotor.Zone.FEEDER_SENSORS.getNearestSlot(rotor.slot0Position, filledSlot);
        if (first == -1) // no Artifacts in the container
            return;

        feedingOrder.add(first);

        int signOfFirstError = (int) signum(Rotor.Zone.FEEDER_SENSORS.distFrom(rotor.slot0Position, first));
        if (signOfFirstError == 0) signOfFirstError = 1;

        int second = wrap(first - signOfFirstError, 0, 3);
        if (artifacts[second] != EMPTY)
            feedingOrder.add(second);

        int third = wrap(second - signOfFirstError, 0, 3);
        if (artifacts[third] != EMPTY)
            feedingOrder.add(third);
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("HANDLER", Arrays.toString(artifacts));
        telemetry.addLine();
        telemetry.addData("Front dist (mm)", front1.getReading());
        telemetry.addLine();
        telemetry.addData("Feeding order", feedingOrder.toString());
        telemetry.addLine("\n--------------------------------------\n");
        rotor.printTo(telemetry);
    }

}
