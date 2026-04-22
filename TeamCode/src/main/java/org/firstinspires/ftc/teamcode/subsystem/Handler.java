package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

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

@Config
public final class Handler {

    public static double
            THRESHOLD_FRONT_MM = 65, // start of ramp = ~115
            INTAKE_POWER_OMNI_CONTACT = 0.4,

            TIME_FRONT_DIST_COOLDOWN = 0.1,
            TIME_FEED = 0.35,

            CACHE_THRESHOLD_INTAKE = 0.05,
            CACHE_THRESHOLD_FEEDER = 0.05;

    private final Rotor rotor;
    private final CachedMotorEx intake;
    private final CachedDcMotor[] feeder;
    private final AnalogSensor frontDistance;

    private double intakePower;
    public void setIntake(double power) {
        this.intakePower = power;
    }

    private double manualFeederPower;
    public void setFeederManual(double power) {
        this.manualFeederPower = power;
    }

    private final ArrayList<Integer> feedingOrder = new ArrayList<>(4);
    private final ElapsedTime timeSinceIntaked = new ElapsedTime(), loopTimer = new ElapsedTime();
    private double timeSpentFeeding;
    private boolean started;

    public static final boolean[] FULL = {true, true, true}, EMPTY = {false, false, false};
    private final boolean[] artifacts = EMPTY.clone();

    public boolean hasArtifacts() {
        return artifacts[0] || artifacts[1] || artifacts[2];
    }
    public boolean isFull() {
        return artifacts[0] && artifacts[1] && artifacts[2];
    }

    public String getContents() {
        return (artifacts[0] ? "O" : "_") + " " + (artifacts[1] ? "O" : "_") + " " + (artifacts[1] ? "O" : "_");
    }

    public void setContents(boolean[] artifacts) {
        this.artifacts[0] = artifacts[0];
        this.artifacts[1] = artifacts[1];
        this.artifacts[2] = artifacts[2];
        feedFastest();
    }

    public void copyContentsTo(boolean[] artifacts) {
        artifacts[0] = this.artifacts[0];
        artifacts[1] = this.artifacts[1];
        artifacts[2] = this.artifacts[2];
    }

    Handler(HardwareMap hardwareMap) {

        rotor = new Rotor(hardwareMap);

        frontDistance = new AnalogSensor(hardwareMap, "front 1", 4000);

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

        int nearestEmptySlot = Rotor.Zone.INTAKE_SENSOR.getNearestSlot(rotor.slot0Position, artifacts, false);

        if (intakePower > 0 && nearestEmptySlot != -1) // move empty slot to intake
            rotor.moveSlot(nearestEmptySlot, Rotor.Zone.INTAKE_SENSOR);
        else if (!feedingOrder.isEmpty())
            rotor.moveSlot(feedingOrder.get(0), Rotor.Zone.FEEDER);

        if (
                nearestEmptySlot != -1 && // there is an empty slot
                Rotor.Zone.INTAKE_SENSOR.slotIsHere(rotor.slot0Position, nearestEmptySlot) && // it is at the front
                frontDistance.getReading() < THRESHOLD_FRONT_MM && // there is something in front of the distance sensor
                timeSinceIntaked.seconds() >= TIME_FRONT_DIST_COOLDOWN // long enough for distance sensor to refresh
        ) {
            artifacts[nearestEmptySlot] = true;
            timeSinceIntaked.reset();
            feedFastest();
        }

        int backSlot = Rotor.Zone.FEEDER.getFilledSlotHere(rotor.slot0Position, Handler.FULL);

        boolean backSlotIsFeedTarget = !feedingOrder.isEmpty() && backSlot == feedingOrder.get(0);

        if (!backSlotIsFeedTarget || !started) {
            timeSpentFeeding = 0; // if slot moves, restart feeding count
            started = true;
        } else if (feed && (timeSpentFeeding += loopTimer.seconds()) >= TIME_FEED) {
            artifacts[backSlot] = false;
            feedingOrder.remove(0);
            backSlotIsFeedTarget = false;
            timeSpentFeeding = 0;
        }
        loopTimer.reset();

        double feederPower =
                manualFeederPower != 0 ? manualFeederPower :
                feed && !feedingOrder.isEmpty() && (backSlotIsFeedTarget || backSlot == -1 || !artifacts[backSlot]) ? 1 : 0;

        for (CachedDcMotor servo : feeder) {
            servo.threshold = CACHE_THRESHOLD_FEEDER;
            servo.setPower(feederPower);
        }

        boolean keepArtifactInside = intakePower >= 0 && intakePower < INTAKE_POWER_OMNI_CONTACT && (
                        Rotor.Zone.INTAKE_OMNI.getFilledSlotHere(rotor.slot0Position, artifacts) != -1 ||
                        Rotor.Zone.INTAKE_OMNI.getFilledSlotHere(rotor.slot0Target, artifacts) != -1
        );

        intake.threshold = CACHE_THRESHOLD_INTAKE;
        intake.set(keepArtifactInside ? INTAKE_POWER_OMNI_CONTACT : intakePower);

        rotor.run();
    }

    boolean feedsPending() {
        return !feedingOrder.isEmpty();
    }

    private void feedFastest() {
        feedingOrder.clear();

        int first = Rotor.Zone.FEEDER.getNearestSlot(rotor.slot0Position, artifacts, true);
        if (first == -1) // no Artifacts in the container
            return;
        
        int second = (first + 1) % 3, third = (first + 2) % 3;
        boolean hasSecond = artifacts[second], hasThird = artifacts[third];

        if (!hasSecond && hasThird) {
            int temp = second;
            second = third;
            third = temp;

            hasSecond = true;
            hasThird = false;
        }

        feedingOrder.add(first);
        feedingOrder.add(second);
        
        if (hasSecond) {
            feedingOrder.add(third);
            if (hasThird)
                feedingOrder.add(first);
        }
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("HANDLER", getContents());
        telemetry.addLine();
        telemetry.addData("Feeding order", feedingOrder);
        telemetry.addData("Time spent feeding", timeSpentFeeding);
        telemetry.addLine();
        telemetry.addData("Front dist (mm)", frontDistance.getReading());
        telemetry.addLine("\n--------------------------------------\n");
        rotor.printTo(telemetry);
    }

}
