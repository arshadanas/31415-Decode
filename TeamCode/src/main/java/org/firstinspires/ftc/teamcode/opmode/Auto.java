package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.Auto.AutoConfig.CONFIRMING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutoConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutoConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.DRIVING_TO_SUB;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.PARKING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_PRELOAD;
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
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.subsystem.Artifact;
import org.firstinspires.ftc.teamcode.subsystem.Motif;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedDcMotor;

@Config
@Autonomous(preselectTeleOp = "Tele")
public final class Auto extends LinearOpMode {

    enum State {
        SCORING_PRELOAD,
        INTAKING_2,
        SCORING,
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
                        2 * SIZE_TILE + Auto.WIDTH_DRIVETRAIN / 2.0,
                        Auto.LENGTH_DRIVETRAIN / 2.0,
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

            TIME_MAX_SPIKE = 1.5,
            TIME_MAX_GATE = 1.5,
            TIME_MAX_SCORE = 1.5;

    public static EditablePose
            admissibleError = new EditablePose(1, 1, 0.05),
            admissibleVel = new EditablePose(25, 25, toRadians(30)),

            scoringPreload = new EditablePose(48.7669, 89.8697, toRadians(-43.3712)),

            startSpike2 = new EditablePose(40, 57.6353, PI),
            intaked2 = new EditablePose(20, 57.6353, PI),

            startSpike3 = new EditablePose(40, 35.0474, PI),
            intaked3 = new EditablePose(20, 35.0474, PI),

            startSpike1 = new EditablePose(40, 82.76, PI),
            intaked1 = new EditablePose(27, 82.76, PI),

            intakingGate = new EditablePose(16, 58, toRadians(167.4)),
            scoring = new EditablePose(-56, 77, toRadians(-130)),

            park = new EditablePose(2 * SIZE_TILE, 3 * SIZE_TILE, toRadians(-130));

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
        }

        robot.setAlliance(isRedAlliance);
        sharedPose = Auto.getStartingPose(isRedAlliance, isGoalSide);
        robot.handler.container.setArtifacts(Motif.PGP.artifacts);

        Action trajectory;

        if (isGoalSide) {

            trajectory = new Action() {

                State state = SCORING_PRELOAD;

                ElapsedTime matchTimer = null;

//                Action activeTraj = scorePreload;

                int subCycle = 1;

                public boolean run(@NonNull TelemetryPacket p) {
                    if (matchTimer == null) matchTimer = new ElapsedTime();

//                    double remaining = (30 - DEAD_TIME) - matchTimer.seconds();

//                    boolean trajDone = !activeTraj.run(p);

//                    switch (state) {
//                        case SCORING_PRELOAD:
//                            robot.headlight.setActivated(true);
//                            if (trajDone) {
//                                state = TAKING_PICTURE;
//                                timer.reset();
//                                sampleDetector.detections.clear();
//                                activeTraj = new SequentialAction(
//                                        new SleepAction(LL_MIN_PICTURE_TIME),
//                                        new FirstTerminateAction(
//                                                t -> !sampleDetector.run(),
//                                                new SleepAction(LL_MAX_PICTURE_TIME)
//                                        )
//                                );
//                            }
//                            break;
//                        case TAKING_PICTURE:
//
//                            if (!sampleDetector.detections.isEmpty()) {
//
//                                double extendoInches = hypot(sampleDetector.offsetToSample.x, sampleDetector.offsetToSample.y) + LL_EXTEND_OFFSET;
//
//                                robot.intake.extendo.powerCap = LL_SPEED_MAX_EXTENDO;
//
//                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
//                                        .turn(-sampleDetector.offsetToSample.heading * LL_TURN_MULTIPLIER)
//                                        .stopAndAdd(() -> {
//                                            robot.intake.extendo.setTarget(extendoInches);
//                                            robot.intake.setAngle(0.01);
//                                            robot.intake.setRoller(0);
//                                        })
//                                        .stopAndAdd(new FirstTerminateAction(
//                                                t -> robot.intake.extendo.getPosition() < extendoInches - LL_DISTANCE_START_LOWERING,
//                                                new SleepAction(1)
//                                        ))
//                                        .stopAndAdd(() -> robot.intake.setRoller(1))
//                                        .stopAndAdd(timer::reset)
//                                        .afterTime(0, t -> !robot.intake.setAngle(timer.seconds() * LL_ANGLE_BUCKET_INCREMENT))
//                                        .waitSeconds(LL_WAIT_INTAKE)
//                                        .build();
//
//                                state = SUB_INTAKING;
//
//                            } else if (trajDone) searchAgainForSample(robot);
//
//                            break;
//
//                        case SUB_INTAKING:
//
//                            if (robot.hasSample()) {
//                                robot.headlight.setActivated(false);
//
//                                robot.intake.extendo.powerCap = 1;
//
//                                activeTraj = deliverSub;
//
//                                subCycle++;
//                                state = DRIVING_TO_SUB;
//                                stopDt();
//
//                            } else if (trajDone) searchAgainForSample(robot);
//
//                            break;
//
//                        case DRIVING_TO_SUB:
//
//                            if (trajDone) {
//                                activeTraj = intake1;
//                                state = INTAKING_1;
//                            }
//                            break;
//
//                        case INTAKING_1:
//
//                            // Sample intaked
//                            if (robot.intake.hasSample()) {
//                                activeTraj = deliver1;
//                                state = SCORING_1;
//                                stopDt();
//                            }
//                            else if (trajDone) { // skip to 2 if didn't get 1
//                                activeTraj = robot.drivetrain.actionBuilder(specIntaking1.toPose2d())
//                                        .stopAndAdd(() -> robot.intake.extendo.setTarget(PRE_EXTEND_SPEC_2))
//                                        .strafeToLinearHeading(specIntaking2.toVector2d(), specIntaking2.heading)
//                                        .stopAndAdd(() -> robot.intake.extendo.setTarget(EXTEND_SPEC_2))
//                                        .stopAndAdd(new FirstTerminateAction(
//                                                t -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SPEC_2)),
//                                                new SleepAction(WAIT_EXTEND_MAX_SPEC_INTAKE)
//                                        ))
//                                        .waitSeconds(WAIT_MAX_INTAKE)
//                                        .build();
//                                state = INTAKING_2;
//                                robot.intake.extendo.setExtended(false);
//                                robot.intake.ejectSample();
//                            }
//
//                            break;
//
//                        case SCORING_1:
//
//                            if (trajDone) {
//                                activeTraj = intake2;
//                                state = INTAKING_2;
//                            }
//                            break;
//
//                        case INTAKING_2:
//
//                            // Sample intaked
//                            if (robot.intake.hasSample()) {
//                                activeTraj = deliver2;
//                                state = SCORING_2;
//                                stopDt();
//                            }
//                            else if (trajDone) { // skip to 3 if didn't get 2
//                                activeTraj = robot.drivetrain.actionBuilder(specIntaking2.toPose2d())
//                                        .stopAndAdd(() -> robot.intake.extendo.setTarget(PRE_EXTEND_SPEC_3))
//                                        .strafeToLinearHeading(specIntaking3.toVector2d(), specIntaking3.heading)
//                                        .stopAndAdd(() -> robot.intake.extendo.setTarget(EXTEND_SPEC_3))
//                                        .stopAndAdd(new FirstTerminateAction(
//                                                t -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SPEC_3)),
//                                                new SleepAction(WAIT_EXTEND_MAX_SPEC_INTAKE)
//                                        ))
//                                        .waitSeconds(WAIT_MAX_INTAKE)
//                                        .build();
//                                state = INTAKING_3;
//                                robot.intake.extendo.setExtended(false);
//                                robot.intake.ejectSample();
//                            }
//
//                            break;
//
//                        case SCORING_2:
//
//                            if (trajDone) {
//                                activeTraj = intake3;
//                                state = INTAKING_3;
//                            }
//                            break;
//
//                        case INTAKING_3:
//
//                            // Sample intaked
//                            if (robot.intake.hasSample() || trajDone) {
//                                activeTraj = scoreAll;
//                                state = SCORING;
//                                stopDt();
//                            }
//
//                            break;
//
//                        case SCORING:
//                            return !trajDone;
//                    }

                    return true;
                }

                private void searchAgainForSample(Robot robot) {
//                    robot.intake.setRollerAndAngle(0);
//                    robot.intake.extendo.setExtended(false);
//                    robot.intake.ejectSample();
//
//                    activeTraj = new SleepAction(WAIT_MAX_BEFORE_RE_SEARCH);
//
//                    state = SCORING_PRELOAD;
                }
            };

        } else {

//            robot.deposit.preloadSample();
//
//            robot.intake.retractBucketBeforeExtendo = false;
//            robot.deposit.requireDistBeforeLoweringLift = false;
//
//            pose = new Pose2d(0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE, 0.5 * WIDTH_ROBOT - SIZE_HALF_FIELD, 0);
//
//            MinVelConstraint inchingConstraint = new MinVelConstraint(Arrays.asList(
//                    new TranslationalVelConstraint(VEL_INCHING),
//                    new AngularVelConstraint(VEL_ANG_INCHING)
//            ));
//
//            robot.deposit.setWristPitchingAngle(ANGLE_PITCH_SPIKES);
//
//            // wait until deposit in position
//            Action scorePreload = robot.drivetrain.actionBuilder(pose)
////                    .afterTime(0, preExtend(robot))
//                    .afterTime(WAIT_BEFORE_SCORING_PRELOAD, scoreSample(robot))
//                    .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
//                    .waitSeconds(WAIT_BARNACLE_DETECT)
//                    .build();
//
//            Action intake1 = new SequentialAction(
//                    robot.drivetrain.actionBuilder(intaking2.toPose2d())
//                            .afterTime(0, preExtend(robot))
//                            .strafeToLinearHeading(intaking1.toVector2d(), intaking1.heading)
//                            .build(),
//                    new InstantAction(() -> {
//                        robot.intake.setRollerAndAngle(1);
//                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_1);
//                    }),
//                    new FirstTerminateAction(
//                            t -> !robot.intake.extendo.atPosition(EXTEND_SAMPLE_1),
//                            t -> !robot.intake.hasSample(),
//                            new SleepAction(WAIT_EXTEND_MAX_SPIKE)
//                    ),
//                    new SleepAction(WAIT_MAX_INTAKE)
//            );
//
//            Action score1 = robot.drivetrain.actionBuilder(intaking1.toPose2d())
//                    .afterTime(0, preExtend(robot))
//                    .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
//                    .stopAndAdd(scoreSample(robot))
//                    .build();
//
//            Action intake2 = new SequentialAction(
//                    new InstantAction(() -> {
//                        robot.intake.setRollerAndAngle(1);
//                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
//                    }),
//                    new FirstTerminateAction(
//                            t -> !robot.intake.extendo.atPosition(EXTEND_SAMPLE_2),
//                            t -> !robot.intake.hasSample(),
//                            new SleepAction(WAIT_EXTEND_MAX_SPIKE)
//                    ),
//                    new SleepAction(WAIT_MAX_INTAKE)
//            );
//
//            Action intake3 = robot.drivetrain.actionBuilder(intaking2.toPose2d())
//                    .strafeToLinearHeading(intaking3.toVector2d(), intaking3.heading, new AngularVelConstraint(VEL_ANG_INTAKING_3))
//                    .stopAndAdd(() -> {
//                        robot.intake.setRollerAndAngle(1);
//                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
//                    })
//                    .stopAndAdd(new FirstTerminateAction(
//                            t -> !robot.intake.extendo.atPosition(EXTEND_SAMPLE_3),
//                            t -> !robot.intake.hasSample(),
//                            new SleepAction(WAIT_EXTEND_MAX_SPIKE)
//                    ))
//                    .setTangent(PI / 2)
//                    .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
//                    .build();

            trajectory = new Action() {

                State state = SCORING_PRELOAD;

                ElapsedTime matchTimer = null;

//                Action activeTraj = scorePreload;

                int subCycle = 1, barnacle = 0;

                void stopDt() {
                    for (CachedDcMotor motor : dtMotors) motor.setPower(0);
                }

                public boolean run(@NonNull TelemetryPacket p) {
                    if (matchTimer == null) matchTimer = new ElapsedTime();

                    double remaining = (30 - DEAD_TIME) - matchTimer.seconds();

//                    boolean trajDone = !activeTraj.run(p);

//                    switch (state) {
//                        case SCORING_PRELOAD:
//
////                            sampleDetector.run();
//
//                            if (trajDone) {
//
////                                if (!sampleDetector.detections.isEmpty()) {
////                                    double degMinSpike1 = (DEG_CENTER + DEG_SPIKE_1) / 2;
////                                    double degMaxSpike2 = (DEG_SPIKE_2 + DEG_CENTER) / 2;
////                                    barnacle =
////                                            sampleDetector.xDegFromLens > degMinSpike1 ? 2 :
////                                            sampleDetector.xDegFromLens < degMaxSpike2 ? 1 :
////                                                                                         3;
////                                }
////
////                                sampleDetector.setPipeline(isRedAlliance ? YELLOW_RED : YELLOW_BLUE);
//                                stopDt();
//
//                                if (barnacle == 1) {
//                                    activeTraj = intake2;
//                                    state = INTAKING_2;
//                                } else {
//                                    activeTraj = intake1;
//                                    state = INTAKING_1;
//                                }
//                            }
//                            break;
//
//                        case INTAKING_1:
//
//                            // Sample intaked
//                            if (robot.intake.hasSample()) {
//                                robot.intake.setRollerAndAngle(0);
//                                activeTraj = score1;
//                                state = SCORING_1;
//                                stopDt();
//                            }
//                            else if (trajDone) { // skip to 2 if didn't get 1
//                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
//                                        .afterTime(0, () -> robot.intake.extendo.setExtended(false))
//                                        .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
//                                        .stopAndAdd(() -> {
//                                            robot.intake.setRollerAndAngle(1);
//                                            robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
//                                        })
//                                        .stopAndAdd(new FirstTerminateAction(
//                                                t -> !robot.intake.extendo.atPosition(EXTEND_SAMPLE_2),
//                                                t -> !robot.intake.hasSample(),
//                                                new SleepAction(WAIT_EXTEND_MAX_SPIKE)
//                                        ))
//                                        .waitSeconds(WAIT_MAX_INTAKE)
////                                        .setTangent(scoring1Intaking2Scoring2.heading)
////                                        .lineToY(scoring1Intaking2Scoring2.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
//                                        .build();
//                                state = INTAKING_2;
//                                robot.intake.extendo.setExtended(false);
//                                robot.intake.ejectSample();
//                            }
//
//                            break;
//
//                        case SCORING_1:
//                            if (trajDone) {
//                                if (barnacle == 2) {
//                                    activeTraj = intake3;
//                                    state = INTAKING_3;
//                                } else {
//                                    activeTraj = intake2;
//                                    state = INTAKING_2;
//                                }
//                            }
//                            break;
//                        case INTAKING_2:
//
//                            // Sample intaked
//                            if (robot.intake.hasSample()) {
//                                robot.intake.setRollerAndAngle(0);
//                                activeTraj = new ParallelAction(
//                                        barnacle == 3 ? new NullAction() : preExtend(robot),
//                                        scoreSample(robot)
//                                );
//                                state = SCORING_2;
//                                stopDt();
//                            }
//                            else if (trajDone) { // skip to 3 if didn't get 2
//                                activeTraj = robot.drivetrain.actionBuilder(intaking2.toPose2d())
//                                        .strafeToLinearHeading(intaking3.toVector2d(), intaking3.heading)
//                                        .stopAndAdd(() -> {
//                                            robot.intake.setRollerAndAngle(1);
//                                            robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
//                                        })
//                                        .stopAndAdd(new FirstTerminateAction(
//                                                t -> !robot.intake.extendo.atPosition(EXTEND_SAMPLE_3),
//                                                t -> !robot.intake.hasSample(),
//                                                new SleepAction(WAIT_EXTEND_MAX_SPIKE)
//                                        ))
//                                        .setTangent(intaking3.heading)
//                                        .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
//                                        .build();
//                                state = INTAKING_3;
//                                robot.intake.extendo.setExtended(false);
//                                robot.intake.ejectSample();
//                            }
//
//                            break;
//
//                        case SCORING_2:
//                            if (trajDone) {
//
//                                if (barnacle == 3) {
//                                    Pose2d current = robot.drivetrain.pose;
//                                    activeTraj = robot.drivetrain.actionBuilder(current)
//                                            .setTangent(current.heading.toDouble())
//                                            .splineTo(snapshotPos.toVector2d(), snapshotPos.heading)
//                                            .build();
//                                    state = DRIVING_TO_SUB;
//                                } else {
//                                    activeTraj = intake3;
//                                    state = INTAKING_3;
//                                }
//                            }
//                            break;
//                        case INTAKING_3:
//
//                            // Sample intaked
//                            if (robot.intake.hasSample()) {
//                                robot.intake.setRollerAndAngle(0);
//                                activeTraj = barnacle == 1 || barnacle == 2 ?
//                                        robot.drivetrain.actionBuilder(intaking3.toPose2d())
//                                                .stopAndAdd(new InstantAction(() -> robot.intake.setRollerAndAngle(0)))
//                                                .strafeToLinearHeading(scoringFromSub.toVector2d(), scoringFromSub.heading)
//                                                .stopAndAdd(scoreSample(robot))
//                                                .build() :
//                                        robot.drivetrain.actionBuilder(intaking3.toPose2d())
//                                                .stopAndAdd(new InstantAction(() -> robot.intake.setRollerAndAngle(0)))
//                                                .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
//                                                .stopAndAdd(scoreSample(robot))
//                                                .build();
//                                state = SCORING;
//                                stopDt();
//                            }
//                            else if (trajDone) { // skip to sub if didn't get 3
//                                activeTraj = barnacle == 1 ?
//                                        robot.drivetrain.actionBuilder(intaking3.toPose2d())
//                                                .strafeToLinearHeading(scoringFromSub.toVector2d(), scoringFromSub.heading)
//                                                .setTangent(scoringFromSub.heading)
//                                                .splineTo(avoidBarnacle1.toVector2d(), avoidBarnacle1.heading)
//                                                .splineTo(snapshotPos.toVector2d(), snapshotPos.heading)
//                                                .build() :
//                                        barnacle == 2 ?
//                                        robot.drivetrain.actionBuilder(intaking3.toPose2d())
//                                                .strafeToLinearHeading(scoringFromSub.toVector2d(), scoringFromSub.heading)
//                                                .setTangent(scoringFromSub.heading)
//                                                .splineTo(snapshotPos.toVector2d(), snapshotPos.heading)
//                                                .build() :
//                                        robot.drivetrain.actionBuilder(intaking3.toPose2d())
//                                                .setTangent(PI / 4)
//                                                .splineToSplineHeading(snapshotPos.toPose2d(), snapshotPos.heading)
//                                                .build();
//                                state = DRIVING_TO_SUB;
//                                robot.intake.setRollerAndAngle(0);
//                                robot.intake.ejectSample();
//                                robot.intake.extendo.setExtended(false);
//                            }
//
//                            break;
//
//                        case SCORING:
//                            if (trajDone) {
//                                Pose2d current = robot.drivetrain.pose;
//                                if (remaining < TIME_CYCLE) {
//
//                                    activeTraj = barnacle == 1 ?
//                                            robot.drivetrain.actionBuilder(current)
//                                                            .setTangent(current.heading)
//                                                            .splineTo(park1.toVector2d(), 0)
//                                                            .build():
//                                            robot.drivetrain.actionBuilder(current)
//                                                    .strafeToLinearHeading((barnacle == 2 ? park2 : park3).toVector2d(), park2.heading)
//                                                    .build();
//                                    state = PARKING;
//                                    stopDt();
//                                } else {
//                                    activeTraj = barnacle == 1 ?
//                                            robot.drivetrain.actionBuilder(current)
//                                                    .setTangent(current.heading.toDouble())
//                                                    .splineTo(avoidBarnacle1.toVector2d(), avoidBarnacle1.heading)
//                                                    .splineTo(snapshotPos.toVector2d(), snapshotPos.heading)
//                                                    .build() :
//                                            robot.drivetrain.actionBuilder(current)
//                                                    .setTangent(current.heading.toDouble())
//                                                    .splineTo(snapshotPos.toVector2d(), snapshotPos.heading)
//                                                    .build();
//                                    state = DRIVING_TO_SUB;
//                                }
//                            }
//                            break;
//
//                        case DRIVING_TO_SUB:
//                            robot.headlight.setActivated(true);
//                            if (trajDone) {
//                                state = TAKING_PICTURE;
//                                timer.reset();
//                                sampleDetector.detections.clear();
//                                activeTraj = new SequentialAction(
//                                        new SleepAction(LL_MIN_PICTURE_TIME),
//                                        new FirstTerminateAction(
//                                                t -> !sampleDetector.run(),
//                                                new SleepAction(LL_MAX_PICTURE_TIME)
//                                        )
//                                );
//                            }
//                            break;
//                        case TAKING_PICTURE:
//
//                            if (remaining < TIME_PARK) barnaclePark();
//                            else if (!sampleDetector.detections.isEmpty()) {
//
//                                double extendoInches = hypot(sampleDetector.offsetToSample.x, sampleDetector.offsetToSample.y) + LL_EXTEND_OFFSET;
//
//                                robot.intake.extendo.powerCap = LL_SPEED_MAX_EXTENDO;
//
//                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
//                                        .turn(-sampleDetector.offsetToSample.heading * LL_TURN_MULTIPLIER)
//                                        .stopAndAdd(() -> {
//                                            robot.intake.extendo.setTarget(extendoInches);
//                                            robot.intake.setAngle(0.01);
//                                            robot.intake.setRoller(0.001);
//                                        })
//                                        .stopAndAdd(new FirstTerminateAction(
//                                                t -> robot.intake.extendo.getPosition() < extendoInches - LL_DISTANCE_START_LOWERING,
//                                                new SleepAction(1)
//                                        ))
//                                        .stopAndAdd(() -> robot.intake.setRoller(1))
//                                        .stopAndAdd(timer::reset)
//                                        .afterTime(0, t -> !robot.intake.setAngle(timer.seconds() * LL_ANGLE_BUCKET_INCREMENT))
//                                        .waitSeconds(LL_WAIT_INTAKE)
//                                        .build();
//
//                                state = SUB_INTAKING;
//
//                            } else if (trajDone) searchAgainForSample(robot);
//
//                            break;
//
//                        case SUB_INTAKING:
//
//
//                            if (remaining < TIME_PARK) barnaclePark();
//                            else if (robot.hasSample()) {
//                                robot.headlight.setActivated(false);
//                                Pose2d current = robot.drivetrain.pose;
//
//                                robot.deposit.requireDistBeforeLoweringLift = true;
//                                robot.intake.retractBucketBeforeExtendo = true;
//                                robot.intake.extendo.powerCap = 1;
//                                robot.deposit.setWristPitchingAngle(0);
//
//                                activeTraj = barnacle == 1 ?
//                                        robot.drivetrain.actionBuilder(current)
//                                                .stopAndAdd(() -> robot.intake.setRollerAndAngle(0))
//                                                .setTangent(PI + current.heading.toDouble())
//                                                .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
//                                                .splineTo(avoidBarnacle1.toVector2d(), PI + avoidBarnacle1.heading)
//                                                .splineTo(scoringFromSub.toVector2d(), PI + scoringFromSub.heading)
//                                                .afterTime(0, () -> robot.deposit.setWristPitchingAngle(ANGLE_PITCH_FROM_SUB))
//                                                .stopAndAdd(scoreSample(robot))
//                                                .afterTime(0, () -> robot.deposit.setWristPitchingAngle(0))
//                                                .build() :
//                                        robot.drivetrain.actionBuilder(current)
//                                                .stopAndAdd(() -> robot.intake.setRollerAndAngle(0))
//                                                .setTangent(PI + current.heading.toDouble())
//                                                .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
//                                                .splineTo(scoringFromSub.toVector2d(), PI + scoringFromSub.heading)
//                                                .afterTime(0, () -> robot.deposit.setWristPitchingAngle(ANGLE_PITCH_FROM_SUB))
//                                                .stopAndAdd(scoreSample(robot))
//                                                .afterTime(0, () -> robot.deposit.setWristPitchingAngle(0))
//                                                .build();
//
//                                subCycle++;
//                                state = SCORING;
//                                stopDt();
//
//                            } else if (trajDone) searchAgainForSample(robot);
//
//                            break;
//
//                        case PARKING:
//                            robot.deposit.lvl1Ascent = true;
//                            return !trajDone;
//                    }

                    return true;
                }

                private void barnaclePark() {
//                    Pose2d current = robot.drivetrain.pose;

                    EditablePose zone = barnacle == 1 ? park :
                                        barnacle == 2 ? park2 :
                                                        park3;

//                    activeTraj = robot.drivetrain.actionBuilder(current)
//                            .stopAndAdd(() -> robot.intake.setRollerAndAngle(0))
//                            .setTangent(PI + current.heading.toDouble())
//                            .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
//                            .splineTo(zone.toVector2d(), PI + zone.heading)
//                            .build();
                    state = PARKING;
                    stopDt();
                }

                private void searchAgainForSample(Robot robot) {
//                    robot.intake.setRollerAndAngle(0);
//                    robot.intake.extendo.setExtended(false);
//                    robot.intake.ejectSample();
//
//                    activeTraj = new FirstTerminateAction(
//                            t -> !(robot.intake.extendo.getPosition() <= 2),
//                            new SleepAction(WAIT_MAX_BEFORE_RE_SEARCH)
//                    );

                    state = DRIVING_TO_SUB;
                }
            };
        }

        ParallelAction auto = new ParallelAction(
                telemetryPacket -> {
//                    robot.run();
                    sharedPose = robot.drivetrain.getPose();
                    System.arraycopy(robot.handler.container.artifacts, 0, Auto.artifacts, 0, 3);
                    return opModeIsActive();
                },
                trajectory
        );

        printConfig(telemetry, true, 0, selected, isGoalSide);
        telemetry.update();

        waitForStart(); //--------------------------------------------------------------------------------------------------------------------------

        robot.drivetrain.setPose(sharedPose);

        Actions.runBlocking(auto);

//        if (robot.deposit.hasSample()) Tele.holdingSample = true;

        Thread.sleep((long) (DEAD_TIME * 1000));
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
        telemetry.update();
    }

    private static Action scoreSpecimen(Robot robot) {
        return new SequentialAction(
                new SleepAction(WAIT_APPROACH_CHAMBER),
//                telemetryPacket -> !(robot.deposit.state == Deposit.State.AT_CHAMBER && robot.deposit.lift.atPosition(HEIGHT_CHAMBER_HIGH)), // wait until deposit in position
//                new InstantAction(robot.deposit::nextState),
//                telemetryPacket -> robot.deposit.hasSample(), // wait until spec scored
                new SleepAction(WAIT_SCORE_CHAMBER)
        );
    }

    private static Action scoreSample(Robot robot) {
        return new SequentialAction(
//                new InstantAction(() -> {
//                    if (!robot.hasSample()) robot.intake.transfer(NEUTRAL);
//                }),
//                new SleepAction(WAIT_APPROACH_BASKET),
//                t -> !(robot.deposit.basketReady() && abs(robot.deposit.lift.getPosition() - HEIGHT_BASKET_HIGH) <= LIFT_HEIGHT_TOLERANCE),
//                new InstantAction(robot.deposit::nextState),
                new SleepAction(WAIT_SCORE_BASKET)
        );
    }

    private static Action preExtend(Robot robot) {
        return new SequentialAction(
//                t -> robot.intake.hasSample(),
//                t -> !(robot.deposit.state == Deposit.State.STANDBY || robot.deposit.state.ordinal() >= Deposit.State.ARM_MOVING_TO_BASKET.ordinal()),
//                new InstantAction(() -> {
//                    robot.intake.extendo.setTarget(PRE_EXTEND);
//                    robot.intake.setRollerAndAngle(1);
//                })
        );
    }

}
