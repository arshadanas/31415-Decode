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
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

import java.util.ArrayList;

@Config
public final class Handler {

    public static double
            TIME_KEEP_FEEDING_AFTER_LAST = 0;

    public final Container container;
    private final CachedMotorEx intake;
    private final CRServo[] feeder;

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

    private boolean keepRunningAfterLastFeed() {
        return feedingOrder.isEmpty() && timeSinceLastFeed.seconds() <= TIME_KEEP_FEEDING_AFTER_LAST;
    }

    boolean feedsPending() {
        return !feedingOrder.isEmpty() || keepRunningAfterLastFeed();
    }

    Handler(HardwareMap hardwareMap) {

        container = new Container(hardwareMap);

        intake = new CachedMotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intake.setInverted(true);
        intake.setZeroPowerBehavior(FLOAT);

        feeder = new CRServo[]{
                hardwareMap.get(CRServo.class, "feeder R"),
                hardwareMap.get(CRServo.class, "feeder L")
        };
        feeder[0].setDirection(REVERSE);
    }

    void run(boolean inLaunchZone, boolean shooterReady) {

        feedingOrder.removeIf(slot -> container.get(slot) == EMPTY);

        // generate feeding order if intake is running
        if (intakePower != 0)
            if (motifMode)
                feedMotif();
            else
                feedFastest();

        // move empty slot to intake
        if (intakePower != 0 && EMPTY.numOccurrencesIn(container.artifacts) > 0)
            container.moveSlot(container.getNearestIntakeSlot(), Container.Zone.INTAKE_SENSORS);
        // move filled slot to feeder
        else if (!feedingOrder.isEmpty())
            container.moveSlot(feedingOrder.get(0), Container.Zone.FEEDER_SENSORS);


        int slotAtFeeder = container.getSlotAt(Container.Zone.FEEDER_OMNIS);
        double feederPower =
                manualFeederPower != 0 ? manualFeederPower : // manual power takes priority
                        inLaunchZone && shooterReady && // <-- don't feed until we can shoot
                        (
                            !feedingOrder.isEmpty() && (slotAtFeeder == -1 || slotAtFeeder == feedingOrder.get(0)) ||
                            keepRunningAfterLastFeed()
                        )
                         ? 1 : 0;


        if (!feedingOrder.isEmpty() && feederPower != 0)
            timeSinceLastFeed.reset();
        

        for (CRServo servo : feeder)
            servo.setPower(feederPower);

        container.run(intakePower, feederPower);

        intake.set(container.adaptiveClipIntakePower(intakePower));
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
        if (first == -1) // no Artifacts in the container
            return;

        feedingOrder.add(first);
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("HANDLER", !motifMode ?
                "Throughput, ignoring motifs" :
                "Scoring motifs, " + (allowOneWrongInMotifs ? "up to one incorrect color" : "colors must be gxact"));
        telemetry.addLine();
        telemetry.addData("Feeding order", feedingOrder.toString());
        telemetry.addLine();
        telemetry.addLine(numArtifactsScored + String.format(" ARTIFACT%s SCORED", numArtifactsScored == 1 ? "" : "S"));
        telemetry.addLine("\n--------------------------------------\n");
        container.printTo(telemetry);
    }

}
