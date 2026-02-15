package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.control.Ranges.wrap;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedDcMotor;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

import java.util.ArrayList;

@Config
public final class Handler {

    public static double
            TIME_KEEP_FEEDING_AFTER_LAST = 1,
            ANGLE_PRESSER_RETRACTED = 87,
            ANGLE_PRESSER_EXTENDED = 211,
            ANGLE_PRESSER_L_OFFSET = -37,
            SPEED_IDLE_FEEDER = 0,

            CACHE_THRESHOLD_INTAKE = 0.05,
            CACHE_THRESHOLD_FEEDER = 0.05,
            CACHE_THRESHOLD_PRESSERS = 0.05,
            INTAKE_CHECKING_TIME = 0.1;

    public final Container container;
    private final CachedMotorEx intake;
    private final CachedDcMotor[] feeder;
    public final SimpleServoPivot presserR, presserL;
    private final CachedSimpleServo presserRServo, presserLServo;

    private final ElapsedTime intakeTimer = new ElapsedTime();

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

    private double intakePower;
    public void setIntake(double power) {
        this.intakePower = power;
    }

    private double manualFeederPower;
    public void setFeederManual(double power) {
        this.manualFeederPower = power;
    }

    private final ArrayList<Integer> feedingOrder = new ArrayList<>();
    private final ElapsedTime timeSinceLastFeed = new ElapsedTime();

    boolean feedsPending() {
        return !feedingOrder.isEmpty() || timeSinceLastFeed.seconds() <= TIME_KEEP_FEEDING_AFTER_LAST;
    }

    Handler(HardwareMap hardwareMap, Runnable boostRPM) {

        container = new Container(hardwareMap, () -> {
            if (motifMode) feedMotif();
            else feedFastest();
        }, boostRPM);

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

    void run(boolean feed) {

        feedingOrder.removeIf(slot -> container.get(slot) == EMPTY);

        boolean feedsEmpty = feedingOrder.isEmpty();
        if (!feedsEmpty)
            timeSinceLastFeed.reset();

        // move empty slot to intake
        if (intakePower != 0 && EMPTY.numOccurrencesIn(container.artifacts) > 0)
            container.moveSlot(container.getNearestIntakeSlot(), Container.Zone.INTAKE_SENSORS);
        // move filled slot to feeder
        else if (!feedsEmpty)
            container.moveSlot(feedingOrder.get(0), Container.Zone.FEEDER_SENSORS);


        int slotAtFeeder = container.getSlotAt(Container.Zone.FEEDER_OMNIS);
        boolean artifactTouchingFeeder = slotAtFeeder != -1 && container.get(slotAtFeeder) != EMPTY;

        double feederPower;
        if (manualFeederPower != 0){
            feederPower = manualFeederPower;

        }else{
            boolean shouldFeed = feed && (
                    (!feedsEmpty && (
                            slotAtFeeder == -1 ||
                            container.get(slotAtFeeder) == EMPTY ||
                            slotAtFeeder == feedingOrder.get(0)
                    )) || (feedsEmpty && timeSinceLastFeed.seconds() <= TIME_KEEP_FEEDING_AFTER_LAST)
            );
            feederPower = shouldFeed ? 1 : SPEED_IDLE_FEEDER;
        }
//
//        double feederPower = manualFeederPower != 0 ?  manualFeederPower :  // manual power takes priority
//                              ? 1 : SPEED_IDLE_FEEDER;

        
        for (CachedDcMotor servo : feeder) {
            servo.threshold = CACHE_THRESHOLD_FEEDER;
            servo.setPower(feederPower);
        }

        container.run(intakePower, feederPower);

        intake.threshold = CACHE_THRESHOLD_INTAKE;
        intake.set(container.adaptiveClipIntakePower(intakePower));

        presserRServo.threshold = CACHE_THRESHOLD_PRESSERS;
        presserLServo.threshold = CACHE_THRESHOLD_PRESSERS;
        presserR.run();
        presserL.run();
    }

    /**
     * Generate the most efficient feeding order
     */
    public void feedFastest() {
        feedingOrder.clear();

        int first = container.getNearestFeedSlot();
        if (first == -1) // no Artifacts in the container
            return;

        feedingOrder.add(first);

        int signOfFirstError = (int) signum(container.getError(first, Container.Zone.FEEDER_SENSORS));

        int second = wrap(first - signOfFirstError, 0, 3);
        if (container.get(second) != EMPTY)
            feedingOrder.add(second);

        int third = wrap(second - signOfFirstError, 0, 3);
        if (container.get(third) != EMPTY)
            feedingOrder.add(third);
    }

    /**
     * Generate feeding order to score {@link Motif} points
     */
    public void feedMotif() {
        feedingOrder.clear();
        feedingOrder.addAll(randomization.getScoringOrder(allowOneWrongInMotifs, numArtifactsScored, container.artifacts));
    }

    public void feedSingle(Artifact color) {
        feedingOrder.clear();

        int first = container.getNearestFeedSlot(color);
        if (first != -1) // no Artifacts in the container
            feedingOrder.add(first);
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("HANDLER", !motifMode ?
                "Throughput, ignoring motifs" :
                "Scoring motifs, " + (allowOneWrongInMotifs ? "up to one incorrect color" : "colors must be gxact"));
        telemetry.addLine();
        telemetry.addData("Feeding order", feedingOrder.toString());
        telemetry.addData("Randomization", randomization);
        telemetry.addLine();
        telemetry.addLine(numArtifactsScored + String.format(" ARTIFACT%s SCORED", numArtifactsScored == 1 ? "" : "S"));
        telemetry.addLine("\n--------------------------------------\n");
        container.printTo(telemetry);
    }

}
