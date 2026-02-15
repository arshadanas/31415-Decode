package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.Auto.AutoConfig.CONFIRMING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutoConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutoConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_SPIKE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.PARKING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.EMPTY;
import static java.lang.Math.PI;
import static java.lang.Math.ceil;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.FirstTerminateAction;
import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.pedropathing.FollowPathAction;
import org.firstinspires.ftc.teamcode.subsystem.Artifact;
import org.firstinspires.ftc.teamcode.subsystem.Motif;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Config
@Autonomous(preselectTeleOp = "Tele")
public final class Auto extends LinearOpMode {

    enum State {
        SCORING,
        INTAKING_SPIKE,
        INTAKING_GATE,
        PARKING,
    }

    static Pose getStartingPose(boolean isRedAlliance, boolean isGoalSide) {
        Pose pose = isGoalSide ?
                new Pose(
                        17.306, // 1 * SIZE_TILE + Auto.WIDTH_DRIVETRAIN / 2.0,
                        120.3163, // SIZE_FIELD - Auto.LENGTH_DRIVETRAIN / 2.0,
                        toRadians(-38)
                ) :
                new Pose(
                        2 * SIZE_TILE + SIZE_TAB / 2 + Auto.LENGTH_DRIVETRAIN / 2,
                        Auto.WIDTH_INCLUDING_PRESSERS / 2,
                        PI
                );

        return isRedAlliance ? pose.mirror() : pose;
    }

    public static double
            DEAD_TIME = 0,
            LENGTH_DRIVETRAIN = 14.81745,
            WIDTH_DRIVETRAIN = 15.53937,
            WIDTH_INCLUDING_PRESSERS = 17.312992126,
            SIZE_FIELD = 141.5,
            SIZE_TILE = SIZE_FIELD/6,
            SIZE_TAB = 0.8,

            TIME_MAX_SPIKE = 1.5,
            TIME_MAX_GATE = 1.5,
            TIME_MAX_SCORE = 30,

            SPEED_INTAKING = 40;

    public static EditablePose
            admissibleError = new EditablePose(1, 1, 0.05),
            admissibleVel = new EditablePose(25, 25, toRadians(30)),

            toSpike2Control = new EditablePose(51.33762057877812, 56.26045016077167, 0),
            startSpike2 = new EditablePose(40, 57.6353, PI),
            intaked2 = new EditablePose(13, 57.6353, PI),

            startSpike1 = new EditablePose(40, 82.76, PI),
            intaked1 = new EditablePose(16.990353697749192, 83.45453376205788, PI),
            openGateControl = new EditablePose(24.901134887459808, 78.59137733118973, 0),
            openGate = new EditablePose(15.892301929260448, 70.65426591639871, PI/2),

            toSpike3Control = new EditablePose(46.0128617363344, 34.06430868167202, 0),
            startSpike3 = new EditablePose(28.887459807073956, 35.27891125401929, PI),
            intaked3 = new EditablePose(13, 35.0474, PI),

            intakingGate = new EditablePose(16, 58, toRadians(167.4)),
            scoring = new EditablePose(48.7669, 89.8697, -PI/2),

            cornerIntaking = new EditablePose(SIZE_TILE / 2, SIZE_TILE / 2, PI),
            farZoneShooting = new EditablePose(2 * SIZE_TILE, SIZE_TILE/2, PI),

            parkNear = new EditablePose(2 * SIZE_TILE, 3 * SIZE_TILE, toRadians(-130)),
            parkFar = new EditablePose(1.5 * SIZE_TILE, 0.5 * SIZE_TILE, PI);

    static Pose sharedPose = null;
    static boolean isRedAlliance = false;
    static final Artifact[] artifacts = {EMPTY, EMPTY, EMPTY};

    enum AutoConfig {
        CONFIRMING,
        EDITING_ALLIANCE,
        EDITING_SIDE;

        public static final AutoConfig[] selections = values();

        public AutoConfig plus(int i) {
            int max = selections.length;
            return selections[((ordinal() + i) % max + max) % max];
        }
        public String markIf(AutoConfig s) {
            return this == s ? "> " : "  ";
        }
    }

    private static boolean feed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // Initialize robot:
        Robot robot = new Robot(hardwareMap, new Pose());

        AutoConfig selected = EDITING_ALLIANCE;
        boolean isGoalSide = false;

        ElapsedTime timer = new ElapsedTime();

        config:
        while (opModeInInit() && timer.seconds() < 5) {

            boolean up = gamepad1.dpadUpWasPressed();
            boolean down = gamepad1.dpadDownWasPressed();
            boolean square = gamepad1.squareWasPressed();

            if (up || down || square) timer.reset();

            if (up)
                selected = selected.plus(-1);
            else if (down)
                selected = selected.plus(1);

            if (square) switch (selected) {
                case CONFIRMING:
                    break config;
                case EDITING_ALLIANCE:
                    isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SIDE:
                    isGoalSide = !isGoalSide;
                    break;
            }

            printConfig(telemetry, false, timer.seconds(), selected, isGoalSide);
        }

        robot.setAlliance(isRedAlliance);
        Pose startPose = sharedPose = Auto.getStartingPose(isRedAlliance, isGoalSide);
        robot.handler.setContents(Motif.PGP.artifacts);

        Follower f = robot.drivetrain.drivetrain;

        Action auto;

        if (!isGoalSide) {

            Pose
                    park = parkFar.toPose(isRedAlliance),
                    cornerIntaking = Auto.cornerIntaking.toPose(isRedAlliance),
                    farZoneShooting = Auto.farZoneShooting.toPose(isRedAlliance);

            auto = new Action() {

                private int timesScored = 0;

                private State state = SCORING;

                private ElapsedTime matchTimer = null;

                // Run the preload action first
                private Action path = shootAll(robot);

                public boolean run(@NonNull TelemetryPacket p) {
                    if (matchTimer == null) matchTimer = new ElapsedTime();

                    double remaining = (30 - DEAD_TIME) - matchTimer.seconds();
                    boolean pathDone = !path.run(p);

                    switch (state) {

                        case SCORING:

                            if (pathDone) {
                                timesScored++;

                                if (timesScored >= 5) {
                                    state = PARKING;
                                    path = new FollowPathAction(f, f.pathBuilder()
                                            .addPath(new BezierLine(startPose, park))
                                            .setConstantHeadingInterpolation(park.getHeading())
                                            .build(), true);
                                } else {
                                    state = INTAKING_SPIKE;
                                    path = new SequentialAction(
                                            new InstantAction(() -> robot.handler.setIntake(1)),
                                            new FollowPathAction(f, f.pathBuilder()
                                                    .addPath(new BezierLine(startPose, cornerIntaking))
                                                    .setConstantHeadingInterpolation(cornerIntaking.getHeading())
                                                    .addPath(new BezierLine(cornerIntaking, farZoneShooting))
                                                    .setConstantHeadingInterpolation(farZoneShooting.getHeading())
                                                    .build(), true),
                                            new InstantAction(() -> robot.handler.setIntake(-0.3))
                                    );
                                }
                            }
                            break;

                        case INTAKING_SPIKE:

                            if (pathDone) {
                                state = SCORING;
                                path = shootAll(robot);
                            }
                            break;

                        case PARKING:
                            if (pathDone)
                                return false;
                            break;
                    }

                    return true;
                }
            };

        } else {

//            if (isRedAlliance)  robot.handler.presserL.setActivated(true);
//            else                robot.handler.presserR.setActivated(true);

            Pose
                    toSpike2Control = Auto.toSpike2Control.toPose(isRedAlliance),
                    openGateControl = Auto.openGateControl.toPose(isRedAlliance),
                    openGate = Auto.openGate.toPose(isRedAlliance),
                    toSpike3Control = Auto.toSpike3Control.toPose(isRedAlliance),
                    startSpike2 = Auto.startSpike2.toPose(isRedAlliance),
                    intaked2 = Auto.intaked2.toPose(isRedAlliance),
                    startSpike3 = Auto.startSpike3.toPose(isRedAlliance),
                    intaked3 = Auto.intaked3.toPose(isRedAlliance),
                    startSpike1 = Auto.startSpike1.toPose(isRedAlliance),
                    intaked1 = Auto.intaked1.toPose(isRedAlliance),
                    intakingGate = Auto.intakingGate.toPose(isRedAlliance),
                    scoring = Auto.scoring.toPose(isRedAlliance),
                    park = Auto.parkNear.toPose(isRedAlliance);

            auto = new Action() {

                /**
                 * IN ORDER OF INTAKING, NOT DELEGATED SPIKE #
                 */
                private int timesScored = 0;

                private State state = SCORING;

                private ElapsedTime matchTimer = null;

                // Run the preload action first
                private Action path = new SequentialAction(
                        new FollowPathAction(f, f.pathBuilder()
                                .addPath(new BezierLine(startPose, scoring))
                                .setLinearHeadingInterpolation(startPose.getHeading(), scoring.getHeading())
                                .build(), false),
                        shootAll(robot)
                );

                public boolean run(@NonNull TelemetryPacket p) {
                    if (matchTimer == null) matchTimer = new ElapsedTime();

                    double remaining = (30 - DEAD_TIME) - matchTimer.seconds();
                    boolean pathDone = !path.run(p);

                    switch (state) {

                        case SCORING:

                            if (pathDone) {
                                timesScored++;

                                if (timesScored == 1) {
                                    state = INTAKING_SPIKE;
                                    path = new SequentialAction(
                                            new InstantAction(() -> robot.handler.setIntake(1)),
                                            new FollowPathAction(f, f.pathBuilder()
                                                    .addPath(new BezierCurve(scoring, toSpike2Control, startSpike2))
                                                    .setLinearHeadingInterpolation(scoring.getHeading(), startSpike2.getHeading())
                                                    .addPath(new BezierLine(startSpike2, intaked2))
                                                    .setConstantHeadingInterpolation(intaked2.getHeading())
                                                    .setVelocityConstraint(SPEED_INTAKING)
                                                    .addPath(new BezierCurve(intaked2, toSpike2Control, scoring))
                                                    .setTangentHeadingInterpolation()
                                                    .setReversed()
                                                    .build(), true
                                            ),
                                            new InstantAction(() -> robot.handler.setIntake(0))
                                    );
                                } else if (timesScored == 2) {
                                    state = INTAKING_SPIKE;
                                    path = new SequentialAction(
                                            new InstantAction(() -> robot.handler.setIntake(1)),
                                            new FollowPathAction(f, f.pathBuilder()
                                                    .addPath(new BezierLine(scoring, startSpike1))
                                                    .setLinearHeadingInterpolation(scoring.getHeading(), startSpike1.getHeading())
                                                    .addPath(new BezierLine(startSpike1, intaked1))
                                                    .setConstantHeadingInterpolation(intaked1.getHeading())
                                                    .setVelocityConstraint(SPEED_INTAKING)
                                                    .addPath(new BezierCurve(intaked1, openGateControl, openGate))
                                                    .setLinearHeadingInterpolation(intaked1.getHeading(), openGate.getHeading())
                                                    .addPath(new BezierLine(openGate, scoring))
                                                    .setLinearHeadingInterpolation(openGate.getHeading(), scoring.getHeading())
                                                    .setReversed()
                                                    .build(), true
                                            ),
                                            new InstantAction(() -> robot.handler.setIntake(0))
                                    );
                                } else if (timesScored == 3) {
                                    state = INTAKING_SPIKE;
                                    path = new SequentialAction(
                                            new InstantAction(() -> robot.handler.setIntake(1)),
                                            new FollowPathAction(f, f.pathBuilder()
                                                    .addPath(new BezierCurve(scoring, toSpike3Control, startSpike3))
                                                    .setTangentHeadingInterpolation()
                                                    .addPath(new BezierLine(startSpike3, intaked3))
                                                    .setConstantHeadingInterpolation(intaked3.getHeading())
                                                    .setVelocityConstraint(SPEED_INTAKING)
                                                    .addPath(new BezierCurve(intaked3, toSpike3Control, scoring))
                                                    .setTangentHeadingInterpolation()
                                                    .setReversed()
                                                    .build(), true
                                            ),
                                            new InstantAction(() -> robot.handler.setIntake(0))
                                    );
                                } else {
                                    if (timesScored >= 4) {
                                        state = PARKING;
                                        path = new FollowPathAction(f, f.pathBuilder()
                                                .addPath(new BezierLine(scoring, park))
                                                .setLinearHeadingInterpolation(scoring.getHeading(), park.getHeading())
                                                .build(), true
                                        );
                                    }
                                }
                            }
                            break;

                        case INTAKING_SPIKE:

                            if (pathDone) {
                                state = SCORING;
                                path = shootAll(robot);
                            }
                            break;

                        case INTAKING_GATE:

                            if (pathDone) {
                                state = SCORING;
                                // update path
                            }
                            break;

                        case PARKING:
                            if (pathDone)
                                return false;
                            break;
                    }

                    return true;
                }
            };
        }

        printConfig(telemetry, true, 0, selected, isGoalSide);

        waitForStart(); //--------------------------------------------------------------------------------------------------------------------------

        robot.drivetrain.setPose(startPose);

        Actions.runBlocking(new ParallelAction(
                telemetryPacket -> {
                    robot.run(feed, false);
                    sharedPose = robot.drivetrain.getPose();
                    System.arraycopy(robot.handler.artifacts, 0, Auto.artifacts, 0, 3);
                    return opModeIsActive();
                },
                auto
        ));

        Thread.sleep((long) (DEAD_TIME * 1000));
    }

    private static Action shootAll(Robot robot) {
        return new SequentialAction(
                new InstantAction(() -> feed = true),
                new FirstTerminateAction(
                        t -> robot.handler.hasArtifacts(),
                        new SleepAction(TIME_MAX_SCORE)
                ),
                new InstantAction(() -> feed = false)
        );
    }

    private static void printConfig(Telemetry telemetry, boolean confirmed, double t, AutoConfig selected, boolean isGoalSide) {
        telemetry.addLine(confirmed ?
                "  AUTONOMOUS READY" :
                CONFIRMING.markIf(selected) + "Confirm (" + (int) ceil(5 - t) + " seconds remaining)"
        );
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine((confirmed ? "  " : EDITING_ALLIANCE.markIf(selected)) + (isRedAlliance ? "RED" : "BLUE") + " alliance");
        telemetry.addLine();
        telemetry.addLine((confirmed ? "  " : EDITING_SIDE.markIf(selected)) + "Starting in " + (isGoalSide ? "near zone (GOAL SIDE)" : "far zone (AUDIENCE SIDE)"));
        telemetry.update();
    }

}
