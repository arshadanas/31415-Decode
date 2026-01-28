package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.control.Wrap.wrap;
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

    private byte numArtifactsScored = 0;
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

        if (intakePower == 0 && power != 0) {// started intaking
            int nearestIntakeSlot = container.getNearestIntakeSlot();
            if (nearestIntakeSlot != -1)
                container.moveSlot(nearestIntakeSlot, Container.Position.INTAKING);
        }

        else if (intakePower != 0 && power == 0) // stopped intaking
            if (motifMode)
                feedMotif();
            else
                feedFastest();

        this.intakePower = power;
    }

    private double manualFeederPower;
    public void setFeederManual(double power) {
        this.manualFeederPower = power;
    }

    private final ArrayList<Integer> feedingOrder = new ArrayList<>();
    private final ElapsedTime keepFeedingAfterLast = new ElapsedTime();

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

    void run(boolean inShootingZone, boolean shooterWheelSpunUp) {

        feedingOrder.removeIf(slot -> container.get(slot) == EMPTY);

        if (!feedingOrder.isEmpty()) { // there is at least one artifact queued to feed
            keepFeedingAfterLast.reset();

            if (intakePower == 0)
                container.moveSlot(feedingOrder.get(0), Container.Position.FEEDING);
        }

        double feederPower =
                manualFeederPower != 0 ? manualFeederPower :
                inShootingZone && shooterWheelSpunUp && (!feedingOrder.isEmpty() || keepFeedingAfterLast.seconds() <= TIME_KEEP_FEEDING_AFTER_LAST) ? 1 : 0;

        for (CRServo servo : feeder)
            servo.setPower(feederPower);

        container.run(feederPower);

        intake.set(container.clipIntakePower(intakePower));
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

        int signOfFirstError = (int) signum(container.getError(first, Container.Position.FEEDING));

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
