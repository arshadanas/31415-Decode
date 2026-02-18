package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.control.Ranges.wrap;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.PURPLE;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedDcMotor;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public final class Handler {

    public static double
            ANGLE_PRESSER_RETRACTED = 87,
            ANGLE_PRESSER_EXTENDED = 211,
            ANGLE_PRESSER_L_OFFSET = -37,

            SPEED_IDLE_FEEDER = 0,
            TIME_KEEP_FEEDING_AFTER_LAST = 0.4,
            THRESHOLD_FRONT_MM = 65, // start of ramp = ~115
            THRESHOLD_BACK_MM = 100, // Height to move onto next feed; above rotor = ~90 // TODO Decrease for faster feeding
            INTAKE_POWER_OMNI_CONTACT = 0.4,
            INTAKE_POWER_IDLE = 0,

            TIME_FRONT_DIST_COOLDOWN = 0.1,
            TIME_FEED_COOLDOWN = 0.2,
            TIME_BACK_DIST_FLUCTUATION = 0.1,

            CACHE_THRESHOLD_INTAKE = 0.05,
            CACHE_THRESHOLD_FEEDER = 0.05,
            CACHE_THRESHOLD_PRESSERS = 0.05;

    public final Rotor rotor;
    private final CachedMotorEx intake;
    private final CachedDcMotor[] feeder;
    private final AnalogSensor front1, back1;
    private final ColorSensor color1, color2;


    public final SimpleServoPivot presserR, presserL;
    private final CachedSimpleServo presserRServo, presserLServo;


    private double intakePower;
    public void setIntake(double power) {
        this.intakePower = power;
    }

    private double manualFeederPower;
    public void setFeederManual(double power) {
        this.manualFeederPower = power;
    }

    private final ArrayList<Integer> feedingOrder = new ArrayList<>();
    private final ElapsedTime
            timeSinceBallAtFeeder = new ElapsedTime(),
            timeSinceHasBall = new ElapsedTime(),
            timeSinceIntaked = new ElapsedTime(),
            timeSinceFeed = new ElapsedTime(),
            timeSinceInFeedingTolerance = new ElapsedTime();

    private final Runnable boostRPM;

    public final Artifact[] artifacts = {EMPTY, EMPTY, EMPTY};

    public boolean isEmpty() {
        return Artifact.EMPTY.numOccurrencesIn(artifacts) == 3;
    }
    public boolean hasArtifacts() {
        return Artifact.EMPTY.numOccurrencesIn(artifacts) < 3;
    }
    public boolean isFull() {
        return Artifact.EMPTY.numOccurrencesIn(artifacts) == 0;
    }

    public void setContents(Artifact[] artifacts) {
        assert artifacts.length == 3;
        if (Arrays.equals(this.artifacts, artifacts))
            return;

        System.arraycopy(artifacts, 0, this.artifacts, 0, 3);
        feedDefault();
    }

    private Artifact a1 = EMPTY, a2 = EMPTY;
    private HSV hsv1 = new HSV(), hsv2 = new HSV();

    Handler(HardwareMap hardwareMap, Runnable boostRPM) {

        rotor = new Rotor(hardwareMap);

        front1 = new AnalogSensor(hardwareMap, "front 1", 4000);
        back1 = new AnalogSensor(hardwareMap, "back 1", 4000);

        color1 = new ColorSensor(hardwareMap, "color 1", 1);
        color2 = new ColorSensor(hardwareMap, "color 2", 1);

        this.boostRPM = boostRPM;

        intake = new CachedMotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intake.setInverted(true);
        intake.setZeroPowerBehavior(FLOAT);

        feeder = new CachedDcMotor[]{
                new CachedDcMotor(hardwareMap.get(CRServo.class, "feeder R")),
                new CachedDcMotor(hardwareMap.get(CRServo.class, "feeder L"))
        };
        feeder[0].motor.setDirection(REVERSE);


        presserRServo = new CachedSimpleServo(hardwareMap, "gate R", 0, 300);
        presserLServo = new CachedSimpleServo(hardwareMap, "gate L", 0, 300).reversed();
        presserLServo.offset = ANGLE_PRESSER_L_OFFSET;

        presserR = new SimpleServoPivot(ANGLE_PRESSER_RETRACTED, ANGLE_PRESSER_EXTENDED, presserRServo);
        presserL = new SimpleServoPivot(ANGLE_PRESSER_RETRACTED, ANGLE_PRESSER_EXTENDED, presserLServo);
    }

    void run(boolean shooterInTolerance) {

        int nearestIntakeSlot = rotor.getNearestSlot(i -> artifacts[i] == EMPTY, Rotor.Zone.INTAKE_SENSORS);
        if (intakePower > 0 && nearestIntakeSlot != -1) // move empty slot to intake
            rotor.moveSlot(nearestIntakeSlot, Rotor.Zone.INTAKE_SENSORS);
        else if (!feedingOrder.isEmpty())
            rotor.moveSlot(feedingOrder.get(0), Rotor.Zone.FEEDER_SENSORS);

        // rotor at position:
        int currentFrontSlot = Rotor.getSlotInTolerance(rotor.slot0Position, Rotor.Zone.INTAKE_SENSORS);
        if (
                currentFrontSlot != -1 && artifacts[currentFrontSlot] == EMPTY && // the slot was previously empty
                front1.getReading() < THRESHOLD_FRONT_MM && // there is something in front of the distance sensor
                timeSinceIntaked.seconds() >= TIME_FRONT_DIST_COOLDOWN
        ) {
            // read i2c
            color1.update();
            color2.update();
            hsv1 = color1.getHSV();
            hsv2 = color2.getHSV();
            a1 = Artifact.fromHSV(hsv1);
            a2 = Artifact.fromHSV(hsv2);

            artifacts[currentFrontSlot] = (a1 == GREEN || a2 == GREEN) ? GREEN : PURPLE;
            timeSinceIntaked.reset();

            feedDefault();
        }

        // Everytime we detect a ball, reset timer
        // This is for the holes
        if (back1.getReading() < THRESHOLD_BACK_MM)
            timeSinceHasBall.reset();

        // check back slot sensors
        int currentBackSlot = Rotor.getSlotInTolerance(rotor.slot0Position, Rotor.Zone.FEEDER_SENSORS);
        if (
                currentBackSlot != -1 && artifacts[currentBackSlot] != EMPTY && // the slot was previously full
                (back1.getReading() >= THRESHOLD_BACK_MM && timeSinceHasBall.seconds() >= TIME_BACK_DIST_FLUCTUATION) && // distance sensor reports no artifact
                timeSinceFeed.seconds() >= TIME_FEED_COOLDOWN
        ) {
            artifacts[currentBackSlot] = EMPTY; // clear the back slot since it has been fed out
            boostRPM.run();
            timeSinceFeed.reset();
        }
        feedingOrder.removeIf(slot -> artifacts[slot] == EMPTY);

        int slotAtFeeder = Rotor.getSlotInTolerance(rotor.slot0Position, Rotor.Zone.FEEDER_OMNIS);
        boolean noBallOrCorrectBallAtFeeder =
                        slotAtFeeder == -1 ||
//                        artifacts[slotAtFeeder] == EMPTY ||
                        !feedingOrder.isEmpty() && slotAtFeeder == feedingOrder.get(0)
        ;

        if (shooterInTolerance && noBallOrCorrectBallAtFeeder)
            timeSinceBallAtFeeder.reset();

        boolean keepRunningFeeder = timeSinceBallAtFeeder.seconds() <= TIME_KEEP_FEEDING_AFTER_LAST;

        double feederPower =
                manualFeederPower != 0 ?                                                    manualFeederPower :
                shooterInTolerance && (noBallOrCorrectBallAtFeeder || keepRunningFeeder) ?  1 :
                                                                                            SPEED_IDLE_FEEDER;

        for (CachedDcMotor servo : feeder) {
            servo.threshold = CACHE_THRESHOLD_FEEDER;
            servo.setPower(feederPower);
        }

        intake.threshold = CACHE_THRESHOLD_INTAKE;
        intake.set(adaptiveClipIntakePower(intakePower));

        rotor.run();

        presserRServo.threshold = CACHE_THRESHOLD_PRESSERS;
        presserLServo.threshold = CACHE_THRESHOLD_PRESSERS;
        presserR.run();
        presserL.run();
    }

    /**
     * Rounds intake speeds of [0, {@link #INTAKE_POWER_OMNI_CONTACT}) up to {@link #INTAKE_POWER_OMNI_CONTACT}
     * if there is an {@link Artifact} touching the intake's front omni wheel <br><br>
     * Rounds speeds of ({@link #INTAKE_POWER_IDLE}, 0] down to {@link #INTAKE_POWER_IDLE}
     * if there is no {@link Artifact} touching the intake's front omni wheel
     */
    double adaptiveClipIntakePower(double intakePower) {
        int omniSlot = Rotor.getSlotInTolerance(rotor.slot0Position, Rotor.Zone.INTAKE_OMNI);
        int slotToFront = Rotor.getSlotInTolerance(rotor.slot0Target, Rotor.Zone.INTAKE_OMNI);
        if (
                intakePower >= 0 && intakePower < INTAKE_POWER_OMNI_CONTACT &&
                (
                        omniSlot != -1 && artifacts[omniSlot] != EMPTY  // ball at front
//                       || slotToFront != -1 && artifacts[slotToFront] != EMPTY // ball going to front
                ) // artifact touching omni wheel
        )
            return INTAKE_POWER_OMNI_CONTACT;

        if (
                intakePower <= 0 && intakePower > INTAKE_POWER_IDLE &&
                (omniSlot == -1 || artifacts[omniSlot] == EMPTY) // no artifact near the omni wheel
        )
            return INTAKE_POWER_IDLE;

        return intakePower;
    }

    boolean feedsPending() {
        return !feedingOrder.isEmpty();
//                || timeSinceFeederRunning.seconds() <= TIME_KEEP_FEEDING_AFTER_LAST;
    }

    private void feedDefault() {
        if (motifMode) feedMotif();
        else feedFastest();
    }

    public void feedSingle(Artifact color) {
        feedingOrder.clear();

        int first = rotor.getNearestSlot(i -> artifacts[i] == color, Rotor.Zone.FEEDER_SENSORS);
        if (first != -1) // no Artifacts in the container
            feedingOrder.add(first);
    }

    /**
     * Generate the most efficient feeding order
     */
    public void feedFastest() {
        feedingOrder.clear();

        int first = rotor.getNearestSlot(i -> artifacts[i] != EMPTY, Rotor.Zone.FEEDER_SENSORS);
        if (first == -1) // no Artifacts in the container
            return;

        feedingOrder.add(first);

        int signOfFirstError = (int) signum(Rotor.getError(first, rotor.slot0Position, Rotor.Zone.FEEDER_SENSORS));

        int second = wrap(first - signOfFirstError, 0, 3);
        if (artifacts[second] != EMPTY)
            feedingOrder.add(second);

        int third = wrap(second - signOfFirstError, 0, 3);
        if (artifacts[third] != EMPTY)
            feedingOrder.add(third);
    }

    /**
     * Generate feeding order to score {@link Motif} points
     */
    public void feedMotif() {
        feedingOrder.clear();
        feedingOrder.addAll(randomization.getScoringOrder(allowOneWrongInMotifs, numArtifactsScored, artifacts));
    }



    public Motif randomization = Motif.PGP;
    /// When scoring 3 {@link Artifact}s intended to score {@link Motif} points, allow up to ONE wrong color {@link Artifact}
    public boolean allowOneWrongInMotifs = false;
    public boolean motifMode = false;

    private int numArtifactsScored = 0;
    public void decrementArtifactsScored() {
        if (numArtifactsScored > 0) numArtifactsScored--;
    }
    public void incrementArtifactsScored() {
        if (numArtifactsScored < 9) numArtifactsScored++;
    }
    public void clearRamp() {
        numArtifactsScored = 0;
    }



    void printTo(Telemetry telemetry) {
        telemetry.addData("HANDLER", Arrays.toString(artifacts));
        telemetry.addData("Empty slot nearest to intake", rotor.getNearestSlot(i -> artifacts[i] == EMPTY, Rotor.Zone.INTAKE_SENSORS));
        telemetry.addData("Filled slot nearest to feeder", rotor.getNearestSlot(i -> artifacts[i] != EMPTY, Rotor.Zone.FEEDER_SENSORS));
        telemetry.addLine();
        telemetry.addData("Artifact 1", a1);
        hsv1.printTo(telemetry);
        telemetry.addLine();
        telemetry.addData("Artifact 2", a2);
        hsv2.printTo(telemetry);
        telemetry.addLine();
        telemetry.addData("Front dist (mm)", front1.getReading());
        telemetry.addData("Back dist (mm)", back1.getReading());
        telemetry.addLine();
        telemetry.addData("Motifs", !motifMode ?
                        "Throughput, ignoring motifs" :
                        "Scoring motifs, " + (allowOneWrongInMotifs ? "up to one incorrect color" : "colors must be gxact")
        );
        telemetry.addLine();
        telemetry.addData("Feeding order", feedingOrder.toString());
        telemetry.addData("Randomization", randomization);
        telemetry.addLine();
        telemetry.addLine(numArtifactsScored + String.format(" ARTIFACT%s SCORED", numArtifactsScored == 1 ? "" : "S"));
        telemetry.addLine("\n--------------------------------------\n");
        rotor.printTo(telemetry);
    }

}
