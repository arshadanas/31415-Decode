package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.control.Wrap.wrap;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static java.lang.Math.max;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

import java.util.ArrayList;

@Config
public final class Handler {

    public final Container container;
    private final CachedMotorEx intake;
    private final CRServo[] feeder;

    public Motif randomization = Motif.PGP;
    /// When scoring 3 {@link Artifact}s intended to score {@link Motif} points, allow up to ONE wrong color {@link Artifact}
    public boolean allowOneWrongInMotifs = false;

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
    public void runIntake(double power) {
        this.intakePower = power;
    }

    private double feederPower;
    public void runFeeder(double power) {
        this.feederPower = power;
    }

    private final ArrayList<Integer> feedingOrder = new ArrayList<>();

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

    void run() {

        for (CRServo servo : feeder)
            servo.setPower(feederPower);

        container.run(feederPower);

        intake.set(
                intakePower < 0 ? intakePower :
                max(intakePower, container.getMinIntakeSpeed())
        );

    }

    /**
     * Generate the most efficient feeding order
     */
    public void feedArbitrary() {
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
        feedingOrder.addAll(randomization.getScoringOrder(allowOneWrongInMotifs, numArtifactsScored, container.getArtifacts()));
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("HANDLER", numArtifactsScored + String.format(" ARTIFACT%s SCORED", numArtifactsScored == 1 ? "" : "S"));
        telemetry.addLine();
        telemetry.addData("Feeding order", feedingOrder.toString());
        telemetry.addLine();
        telemetry.addData("Motifs", allowOneWrongInMotifs ? "Allowing one incorrect Artifact" : "Artifact colors must be gxact");
        telemetry.addLine("\n--------------------------------------\n");
        container.printTo(telemetry);
    }

}
