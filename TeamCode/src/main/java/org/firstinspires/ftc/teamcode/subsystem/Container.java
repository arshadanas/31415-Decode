package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.PURPLE;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.WraparoundKalmanFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.KalmanGains;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.AnalogSensor;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

import java.util.Arrays;

@Config
public final class Container {

    private final WraparoundKalmanFilter derivFilter = new WraparoundKalmanFilter(filterGains);
    public static KalmanGains filterGains = new KalmanGains(15.9, 12);

    private final PIDController controller = new PIDController(derivFilter);
    public static PIDGains pidGains = new PIDGains(0.125, 0, 0.015);

    public static double
            ABS_OFFSET_ROTOR = 1.4013407230558101,
            THRESHOLD_FRONT_MM = 70, // start of ramp = ~115
            THRESHOLD_BACK_MM = 70, // above rotor = ~75
            INTAKE_POWER_OMNI_CONTACT = 0.4,
            INTAKE_POWER_IDLE = -0.2,

            TOLERANCE_FRONT = toRadians(20),
            TOLERANCE_BACK = toRadians(20),
            TOLERANCE_FRICTION = toRadians(35),
            TOLERANCE_FRONT_OMNI = toRadians(30),

            POWER_OVERCOME_FRICTION = 0.06;

    // hardware
    private final CRServo[] servos;
    private final AnalogSensor encoder, front1, back1;
    private final ColorSensor color1, color2;
    private final LEDIndicator[] indicators;

    /**
     * Position of slot 0, in radians
     */
    private double position = 0;

    /**
     * Position of given slot, in radians
     */
    private double getPositionOf(int slot) {
        return normalizeRadians(position + slot * 2 * PI / 3.0);
    }

    final Artifact[] artifacts = {EMPTY, EMPTY, EMPTY};

    private int selectedSlot = 0;
    private Position target = Position.INTAKING;

    enum Position {
        INTAKING(0),
        FEEDING(PI),
        FRICTION_ZONE(PI),
        FRONT_OMNI_ZONE(0);

        private final double radians;
        Position(double radians) {
            this.radians = radians;
        }

        private double getTolerance() {
            switch (this) {
                case FEEDING:           return TOLERANCE_BACK;
                case FRICTION_ZONE:     return TOLERANCE_FRICTION;
                case FRONT_OMNI_ZONE:   return TOLERANCE_FRONT_OMNI;
                default:                return TOLERANCE_FRONT;
            }
        }

    }

    Container(HardwareMap hardwareMap) {
        servos = new CRServo[]{
                hardwareMap.get(CRServo.class, "rotor 1"),
                hardwareMap.get(CRServo.class, "rotor 2")
        };

        encoder = new AnalogSensor(hardwareMap, "rotor", 2 * PI);

        front1 = new AnalogSensor(hardwareMap, "front 1", 1300);
        back1 = new AnalogSensor(hardwareMap, "back 1", 1000);

        color1 = new ColorSensor(hardwareMap, "color 1", 1);
        color2 = new ColorSensor(hardwareMap, "color 2", 1);

        indicators = new LEDIndicator[]{
                new LEDIndicator(hardwareMap, "led 1a", "led 1b"),
                new LEDIndicator(hardwareMap, "led 2a", "led 2b"),
                new LEDIndicator(hardwareMap, "led 3a", "led 3b")
        };
    }

    void run(double feederPower) {

        position = normalizeRadians(encoder.getReading() + ABS_OFFSET_ROTOR);


        int currentFrontSlot = getSlotAt(Position.INTAKING);
        if (
                currentFrontSlot != -1 &&   // there is a slot near the front intaking zone
                artifacts[currentFrontSlot] == EMPTY && // the slot was previously empty
                front1.getReading() < THRESHOLD_FRONT_MM // there is something in front of the distance sensor
        ) {
            // read i2c
            color1.update();
            color2.update();

            // combine Artifact reading from both color sensors
            artifacts[currentFrontSlot] = Artifact.fromHSV(color1.getHSV()). or (Artifact.fromHSV(color2.getHSV()));
        }


        // check back slot sensors
        int currentBackSlot = getSlotAt(Position.FEEDING);
        if (
                currentBackSlot != -1 && // there is a slot near the back feeding zone
                artifacts[currentBackSlot] != EMPTY && // the slot was not previously empty
                feederPower > 0 &&  // the feeder is running
                back1.getReading() > THRESHOLD_BACK_MM // distance sensor reports no artifact
        )
            artifacts[currentBackSlot] = EMPTY; // clear the back slot since it has been fed out


        // LEDs
        int n = 0;
        for (Artifact a : artifacts)
            if (a != EMPTY)
                indicators[n++].setColor(a.toLEDColor());
        while (n < artifacts.length)
            indicators[n++].setColor(EMPTY.toLEDColor());


        // run pid
        derivFilter.setGains(filterGains);
        controller.setGains(pidGains);
        controller.setTarget(new State(getError(selectedSlot, target)));

        double servoPower = controller.calculate(new State());

        int frictionSlot = getSlotAt(Position.FRICTION_ZONE);
        double antiFrictionPower = frictionSlot != -1 && artifacts[frictionSlot] != EMPTY ?
                                    POWER_OVERCOME_FRICTION * signum(servoPower) : 0;

        for (CRServo servo : servos)
            servo.setPower(servoPower + antiFrictionPower);
    }

    /**
     * Rounds intake speeds of [0, {@link #INTAKE_POWER_OMNI_CONTACT}) up to {@link #INTAKE_POWER_OMNI_CONTACT}
     * if there is an {@link Artifact} touching the intake's front omni wheel <br><br>
     * Rounds speeds of ({@link #INTAKE_POWER_IDLE}, 0] down to {@link #INTAKE_POWER_IDLE}
     * if there is no {@link Artifact} touching the intake's front omni wheel
     */
    double adaptiveClipIntakePower(double intakePower) {
        int omniSlot = getSlotAt(Position.FRONT_OMNI_ZONE);
        if (
                intakePower >= 0 &&
                intakePower < INTAKE_POWER_OMNI_CONTACT &&
                omniSlot != -1 && artifacts[omniSlot] != EMPTY // artifact touching omni wheel
        )
            return INTAKE_POWER_OMNI_CONTACT;

        if (
                intakePower <= 0 &&
                intakePower > INTAKE_POWER_IDLE &&
                (omniSlot == -1 || artifacts[omniSlot] == EMPTY) // no artifact near the omni wheel
        )
            return INTAKE_POWER_IDLE;

        return intakePower;
    }

    /**
     * @param slot Slot you wish to move (0, 1 or 2)
     */
    void moveSlot(int slot, Position target) {
        this.selectedSlot = slot;
        this.target = target;
    }

    /**
     * GREEN in slot 1
     */
    public void preloadPGP() {
        artifacts[0] = PURPLE;
        artifacts[1] = GREEN;
        artifacts[2] = PURPLE;
    }

    /**
     * @return Empty slot closest to the intake. -1 if no such slot found
     */
    int getNearestIntakeSlot() {
        double min = Double.MAX_VALUE;
        int minInd = -1;
        for (int i = 0; i < 3; i++) {
            if (artifacts[i] != EMPTY)
                continue;

            double error = getError(i, Position.INTAKING);
            if (error < min){
                min = error;
                minInd = i;
            }
        }
        return minInd;
    }

    /**
     * @return Occupied slot closest to the feeder. -1 if no such slot found
     */
    int getNearestFeedSlot() {
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

    /**
     * @return Slot closest to the feeder, and holding an Artifact of the given color. -1 if no such slot found
     */
    int getNearestFeedSlot(Artifact color) {
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

    Artifact get(int slot) {
        return artifacts[slot];
    }

    /**
     * @return The (index of the) slot currently at the given target, -1 if no slot at that position
     */
    int getSlotAt(Position target) {
        for (int i = 0; i < artifacts.length; i++)
            if (atPosition(i, target))
                return i;
        return -1;
    }

    /**
     * @return  If the given slot is at the given target, within tolerance in either direction
     */
    boolean atPosition(int slot, Position target) {
        return abs(getError(slot, target)) <= target.getTolerance();
    }

    /**
     * @return  Distance, in radians, between given slot's position and given target
     */
    double getError(int slot, Position target) {
        return normalizeRadians(target.radians - getPositionOf(slot));
    }

    void printTo(Telemetry telemetry) {
        telemetry.addData("CONTAINER", Arrays.toString(artifacts));
        telemetry.addLine();
        telemetry.addData(String.format("Current (slot %s) (deg)", selectedSlot), toDegrees(getPositionOf(selectedSlot)));
        telemetry.addData("Target (deg)", toDegrees(target.radians));
        telemetry.addData("Error (deg)", toDegrees(getError(selectedSlot, target)));
        telemetry.addData("Filtered error derivative (deg/s)", toDegrees(controller.getFilteredErrorDerivative()));
        telemetry.addData("Raw error derivative (deg/s)", toDegrees(controller.getRawErrorDerivative()));
        telemetry.addLine();
        telemetry.addData("Position (slot 0) (deg)", toDegrees(position));
        telemetry.addLine();
        telemetry.addData("Front dist (mm)", front1.getReading());
        telemetry.addData("Back dist (mm)", back1.getReading());
    }

}
