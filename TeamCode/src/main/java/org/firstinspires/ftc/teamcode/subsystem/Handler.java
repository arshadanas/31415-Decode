package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.control.Wrap.wrap;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static java.lang.Math.max;
import static java.lang.Math.signum;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

import java.util.ArrayList;

public final class Handler {

    public final Container container;
    private final CachedMotorEx intake;
    private final CRServo[] feeder;

    public Motif randomization;
    /// When scoring 3 {@link Artifact}s intended to score {@link Motif} points, allow up to ONE wrong color {@link Artifact}
    public boolean allowOneWrongInMotifs;

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

    public void feedMotif(boolean allowOneWrong, byte numArtifactsScored) {
        feedingOrder.clear();
        feedingOrder.addAll(randomization.getScoringOrder(allowOneWrong, numArtifactsScored, container.getArtifacts()));
    }

    void printTo(Telemetry telemetry) {
        container.printTo(telemetry);
    }

}
