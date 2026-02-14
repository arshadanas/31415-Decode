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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
                        toRadians(-37.9624)
                ) :
                new Pose(
                        2 * SIZE_TILE + SIZE_TAB / 2 + Auto.LENGTH_DRIVETRAIN / 2,
                        Auto.WIDTH_INCLUDING_PRESSERS / 2,
                        PI / 2
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
            TIME_MAX_SCORE = 3;

    public static EditablePose
            admissibleError = new EditablePose(1, 1, 0.05),
            admissibleVel = new EditablePose(25, 25, toRadians(30)),

            scoringPreload = new EditablePose(48.7669, 89.8697, toRadians(-43.3712)),

            toSpike2Control = new EditablePose(58.51446945337619, 60.19614147909965, 0),
            startSpike2 = new EditablePose(40, 57.6353, PI),
            intaked2 = new EditablePose(20, 57.6353, PI),

            toSpike3Control = new EditablePose(58.51446945337619, 60.19614147909965, 0),
            startSpike3 = new EditablePose(40, 35.0474, PI),
            intaked3 = new EditablePose(20, 35.0474, PI),

            toSpike1Control = new EditablePose(58.51446945337619, 60.19614147909965, 0),
            startSpike1 = new EditablePose(40, 82.76, PI),
            intaked1 = new EditablePose(20, 82.76, PI),

            intakingGate = new EditablePose(16, 58, toRadians(167.4)),
            scoring = new EditablePose(-56, 77, toRadians(-130)),

            parkNear = new EditablePose(2 * SIZE_TILE, 3 * SIZE_TILE, toRadians(-130)),
            parkFar = new EditablePose(1.5 * SIZE_TILE, 0.5 * SIZE_TILE, PI/2);

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
            return this == s ? " <" : "";
        }
    }

    private static boolean feed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
            telemetry.update();
        }

        robot.setAlliance(isRedAlliance);
        sharedPose = Auto.getStartingPose(isRedAlliance, isGoalSide);
        robot.handler.container.setArtifacts(Motif.PGP.artifacts);

        Follower f = robot.drivetrain.drivetrain;

        Action auto;

        if (!isGoalSide) {

            Pose
                    park = parkFar.toPose(isRedAlliance);

            auto = new SequentialAction(
                    shoot3(robot),
                    new FollowPathAction(f, f.pathBuilder()
                            .addPath(new BezierLine(sharedPose, park))
                            .setConstantHeadingInterpolation(park.getHeading())
                            .build(), true)
            );

        } else {

            Pose
                    scoringPreload = Auto.scoringPreload.toPose(isRedAlliance),
                    toSpike2Control = Auto.toSpike2Control.toPose(isRedAlliance),
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
                                .addPath(new BezierLine(sharedPose, scoringPreload))
                                .setLinearHeadingInterpolation(sharedPose.getHeading(), scoringPreload.getHeading())
                                .build(), true),
                        shoot3(robot)
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
                                                    .addPath(new BezierCurve(sharedPose, toSpike2Control, startSpike2))
                                                    .setLinearHeadingInterpolation(sharedPose.getHeading(), startSpike2.getHeading())
                                                    .addPath(new BezierLine(startSpike2, intaked2))
                                                    .setConstantHeadingInterpolation(intaked2.getHeading())
                                                    .build(), true),
                                            new InstantAction(() -> robot.handler.setIntake(0))
                                    );
                                }
                                else if (timesScored >= 2) {
                                    state = PARKING;
                                    path = new FollowPathAction(f, f.pathBuilder()
                                            .addPath(new BezierLine(sharedPose, park))
                                            .setLinearHeadingInterpolation(sharedPose.getHeading(), park.getHeading())
                                            .build(), true);
                                }
                            }
                            break;

                        case INTAKING_SPIKE:

                            if (pathDone) {
                                state = SCORING;
                                Pose control = toSpike2Control;
                                path = new SequentialAction(
                                        new FollowPathAction(f, f.pathBuilder()
                                                .addPath(new BezierCurve(sharedPose, control, scoring))
                                                .setTangentHeadingInterpolation()
                                                .build(), true),
                                        shoot3(robot)
                                );
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
        telemetry.update();

        waitForStart(); //--------------------------------------------------------------------------------------------------------------------------

        robot.drivetrain.setPose(sharedPose);

        Actions.runBlocking(new ParallelAction(
                telemetryPacket -> {
                    robot.run(feed, false);
                    sharedPose = robot.drivetrain.getPose();
                    System.arraycopy(robot.handler.container.artifacts, 0, Auto.artifacts, 0, 3);
                    return opModeIsActive();
                },
                auto
        ));

        Thread.sleep((long) (DEAD_TIME * 1000));
    }

    private static Action shoot3(Robot robot) {
        return new SequentialAction(
                new InstantAction(() -> feed = true),
                new FirstTerminateAction(
                        t -> robot.hasArtifacts(),
                        new SleepAction(TIME_MAX_SCORE)
                ),
                new InstantAction(() -> feed = false)
        );
    }

    private static void printConfig(Telemetry telemetry, boolean confirmed, double t, AutoConfig selected, boolean isGoalSide) {
        telemetry.addLine(confirmed ?
                "AUTONOMOUS READY" :
                "Confirm configuration (confirming in " + (int) ceil(5 - t) + " seconds)" + selected.markIf(CONFIRMING)
        );
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine(EDITING_ALLIANCE.markIf(selected) + (isRedAlliance ? "RED" : "BLUE") + " alliance");
        telemetry.addLine();
        telemetry.addLine(EDITING_SIDE.markIf(selected) + "Starting in " + (isGoalSide ? "near zone (GOAL SIDE)" : "far zone (AUDIENCE SIDE)"));
    }

}
