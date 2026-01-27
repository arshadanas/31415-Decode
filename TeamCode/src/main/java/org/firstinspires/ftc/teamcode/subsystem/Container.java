package org.firstinspires.ftc.teamcode.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Profiles;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeProfile;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.FeedforwardController;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.gainmatrix.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

import java.util.Arrays;

@Config
public final class Container {

    private TimeProfile profile;
    private final ElapsedTime profileTimer = new ElapsedTime();

    public static PIDGains pidGains = new PIDGains(0, 0, 0);

    private final FeedforwardController feedforward = new FeedforwardController();
    public static FeedforwardGains ffGains = new FeedforwardGains(0.01, 0, 0.025);

    private final WrapDiff velo = new WrapDiff();
    private final Differentiator accel = new Differentiator();

    public static LowPassGains vGains = new LowPassGains(0.6, 40), aGains = new LowPassGains();
    private final FIRLowPassFilter vFilter = new FIRLowPassFilter(), aFilter = new FIRLowPassFilter();

    private final PIDController pid = new PIDController(vFilter);

    public static double
            ABS_OFFSET_ROTOR = -1.1366964485759112,
            THRESHOLD_FRONT_MM = 70, // start of ramp = ~115
            THRESHOLD_BACK_MM = 70, // above rotor = ~75
            INTAKE_SPEED_WHEN_SORTING = 0,
            ROTOR_SPEED_THRESHOLD_INTAKE_SPIN = 0.5,

            TOLERANCE_FRONT = toRadians(20),
            TOLERANCE_BACK = toRadians(20),
            TOLERANCE_FRICTION = toRadians(30),

            MAX_VEL = 34.8716784548467,
            MAX_ACCEL = 69.81317007977317,
            MIN_ACCEL = -MAX_ACCEL;

    // hardware
    final CRServo[] servos;
    private final AnalogSensor encoder, frontDist1, backDist1;
    private final ColorSensor color1, color2;
    private final LEDIndicator[] indicators;

    /**
     * Position of slot 0, in radians
     */
    private State currentMeasurement = new State(), profileTarget = new State();

    /**
     * Position of given slot, in radians
     */
    private double getPositionOf(int slot) {
        return normalizeRadians(currentMeasurement.x + slot * 2 * PI / 3.0);
    }

    private final Artifact[] artifacts = {EMPTY, EMPTY, EMPTY};

    private int selectedSlot = 0;
    private Position target = Position.INTAKING;

    enum Position {
        INTAKING(0),
        FEEDING(PI),
        FRICTION(PI);

        private final double radians;
        Position(double radians) {
            this.radians = radians;
        }

        private double getTolerance() {
            switch (this) {
                case FEEDING:   return TOLERANCE_BACK;
                case FRICTION:  return TOLERANCE_FRICTION;
                default:        return TOLERANCE_FRONT;
            }
        }
    }

    Container(HardwareMap hardwareMap) {
        servos = new CRServo[]{
                hardwareMap.get(CRServo.class, "rotor 1"),
                hardwareMap.get(CRServo.class, "rotor 2")
        };

        encoder = new AnalogSensor(hardwareMap, "rotor", 3 * 2 * PI);

        frontDist1 = new AnalogSensor(hardwareMap, "front 1", 1300);
        backDist1 = new AnalogSensor(hardwareMap, "back 1", 1000);

        color1 = new ColorSensor(hardwareMap, "color 1", 1);
        color2 = new ColorSensor(hardwareMap, "color 2", 1);

        indicators = new LEDIndicator[]{
                new LEDIndicator(hardwareMap, "led 1a", "led 1b"),
                new LEDIndicator(hardwareMap, "led 2a", "led 2b"),
                new LEDIndicator(hardwareMap, "led 3a", "led 3b")
        };

        intake = new CachedMotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        intake.setInverted(true);
        intake.setZeroPowerBehavior(FLOAT);

        generateProfile(0);
    }

    private final CachedMotorEx intake;

    private static class WrapDiff {

        private double lastValue = Double.NaN, derivative = 0.0;

        private final ElapsedTime timer = new ElapsedTime();

        public double getDerivative(double newValue) {

            double dt = timer.seconds();
            timer.reset();

            if (dt != 0.0 && !Double.isNaN(lastValue)) {
                derivative = normalizeRadians(newValue - lastValue) / dt;
            }

            lastValue = newValue;

            return derivative;
        }
    }

    void run() {
        // Set filter and feedforward gains
        vFilter.setGains(vGains);
        aFilter.setGains(aGains);
        feedforward.setGains(ffGains);

        double position = normalizeRadians(encoder.getReading() + ABS_OFFSET_ROTOR);
        double velocity = vFilter.calculate(velo.getDerivative(position));
        double acceleration = accel.getDerivative(aFilter.calculate(velocity));

        currentMeasurement = new State(position, velocity, acceleration);

        int currentFrontSlot = getSlotAt(Position.INTAKING);
        if (
                currentFrontSlot != -1 &&
                artifacts[currentFrontSlot] == EMPTY &&
                frontDist1.getReading() < THRESHOLD_FRONT_MM
                // TODO check rotor speed under threshold
        ) {
            color1.update();
            color2.update();
            // combine Artifact reading from both color sensors
            artifacts[currentFrontSlot] = Artifact.fromHSV(color1.getHSV()). or (Artifact.fromHSV(color2.getHSV()));

            if (artifacts[currentFrontSlot] != EMPTY) {
                int nextEmptySlot = EMPTY.firstOccurrenceIn(artifacts);

                if (nextEmptySlot == -1) // no empty slots
                    moveSlot(getNearestFeedSlot(), Position.FEEDING); // move artifact to feeder
                else
                    moveSlot(nextEmptySlot, Position.INTAKING);
            }
        }

        // check back slot sensors
        int currentBackSlot = getSlotAt(Position.FEEDING);
        if (
                currentBackSlot != -1 &&
                artifacts[currentBackSlot] != EMPTY &&
                backDist1.getReading() > THRESHOLD_BACK_MM
                // TODO check rotor speed under threshold
        )
            artifacts[currentBackSlot] = EMPTY;

        // LEDs
        int n = 0;
        for (Artifact a : artifacts)
            if (a != EMPTY)
                indicators[n++].setColor(a.toLEDColor());
        while (n < artifacts.length)
            indicators[n++].setColor(EMPTY.toLEDColor());


        // profile stuff

        // Get the current time in the profile
        double t = profileTimer.seconds();

        // The target state that we are trying to hit at this point in time
        DualNum<Time> profileAtTimeT = profile.get(t);
        profileTarget = new State(normalizeRadians(profileAtTimeT.get(0)), profileAtTimeT.get(1), profileAtTimeT.get(2))
                .times(profileDirection);

        pid.setGains(pidGains);
        pid.setTarget(new State(normalizeRadians(profileTarget.x + profileStartPosition - getPositionOf(selectedSlot))));
        double pidOutput = pid.calculate(new State());

        feedforward.setTarget(profileTarget);
        double ffOutput = feedforward.calculate(pidOutput);

        rotorOutput = pidOutput + ffOutput;

        rotorAboveThreshold = abs(rotorOutput) > ROTOR_SPEED_THRESHOLD_INTAKE_SPIN;
        for (CRServo servo : servos)
            servo.setPower(rotorOutput);

        intake.set(getMinIntakeSpeed());
    }

    private double rotorOutput;

    private boolean artifactIsTouchingFeeder() {
        int frictionSlot = getSlotAt(Position.FRICTION);
        return frictionSlot != -1 && artifacts[frictionSlot] != EMPTY;
    }

    /** //TODO only spin intake if ball near the front
     * When we spin the rotor, if an {@link Artifact} is in the front (TBA), the intake omni wheel must spin to contain the {@link Artifact}
     */
    double getMinIntakeSpeed() {
        return rotorAboveThreshold ? INTAKE_SPEED_WHEN_SORTING : 0;
    }

    void print(Telemetry telemetry) {
        telemetry.addData("CONTAINER", Arrays.toString(artifacts));
        telemetry.addLine();
        telemetry.addData("Current (deg)", toDegrees(getPositionOf(selectedSlot)));
        telemetry.addData("Error (deg)", toDegrees(getError(selectedSlot, target)));
        telemetry.addData("Current output", rotorOutput);
        telemetry.addLine();
        telemetry.addData("Position (slot 0) (rad)", currentMeasurement.x);
        telemetry.addData("Position (slot 0) (deg)", toDegrees(currentMeasurement.x));
        telemetry.addData("Target (slot 0) (deg)", toDegrees(profileStartPosition + getError(selectedSlot, target)));
        telemetry.addData("Velocity (deg/s)", toDegrees(currentMeasurement.v));
        telemetry.addData("Acceleration (deg/s^2)", toDegrees(currentMeasurement.a));
        telemetry.addLine();
        telemetry.addData("Target x (deg)", toDegrees(normalizeRadians(profileStartPosition + profileTarget.x)));
        telemetry.addData("Target v (deg/s)", toDegrees(profileTarget.v));
        telemetry.addData("Target a (deg/s^2)", toDegrees(profileTarget.a));
        telemetry.addLine();
        telemetry.addData("Front dist (mm)", frontDist1.getReading());
        telemetry.addData("Back dist (mm)", backDist1.getReading());
    }

    private boolean rotorAboveThreshold = false;

    private int getNearestFeedSlot() {
        double min = Double.MAX_VALUE;
        int minInd = -1;
        for (int i = 0; i < 3; i++) {
            if (artifacts[i] == EMPTY)
                continue;

            double error = getError(i, Position.FEEDING);
            if (error < min){
                min = error;
                minInd = i;
            }
        }
        return minInd;
    }

    private int getNearestFeedSlot(Artifact color) {
        double min = Double.MAX_VALUE;
        int minInd = -1;
        for (int i = 0; i < 3; i++) {
            if (artifacts[i] != color)
                continue;

            double error = getError(i, Position.FEEDING);
            if (error < min){
                min = error;
                minInd = i;
            }
        }
        return minInd;
    }

    /**
     * @return The (index of the) slot currently at the given target, -1 if no slot at that position
     */
    private int getSlotAt(Position target) {
        for (int i = 0; i < artifacts.length; i++)
            if (atPosition(i, target))
                return i;
        return -1;
    }

    /**
     * @param slot Slot you wish to move (0, 1 or 2)
     */
    void moveSlot(int slot, Position target) {
        this.selectedSlot = slot;
        this.target = target;

        profileStartPosition = getPositionOf(selectedSlot);
        generateProfile(getError(selectedSlot, target));
    }

    private double profileStartPosition;

    private void generateProfile(double distance) {
        if (distance == 0)
            distance = 0.05;

        profileTimer.reset();
        profile = new TimeProfile(Profiles.constantProfile(
                abs(distance),
                0,
                MAX_VEL,
                MIN_ACCEL,
                MAX_ACCEL
        ).baseProfile);

        profileDirection = signum(distance);
    }

    private double profileDirection = 1;

    /**
     * @return  If the given slot is at the given target, within tolerance in either direction
     */
    private boolean atPosition(int slot, Position target) {
        return abs(getError(slot, target)) <= target.getTolerance();
    }

    /**
     * @return  Distance, in radians, between given slot's position and given target
     */
    private double getError(int slot, Position target) {
        return normalizeRadians(target.radians - getPositionOf(slot));
    }

}
