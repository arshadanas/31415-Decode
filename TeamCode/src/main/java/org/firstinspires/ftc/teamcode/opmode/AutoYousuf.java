package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

public class AutoYousuf extends OpMode {

        // Mechanical
        public static double SHOOTER_TRANSFER_DELAY = 650.0;
        public static double INTAKE_RECOLLECTION_TIMEOUT = 500.0;
        public static long INTAKE_STOP_DELAY = 350;

        // Gate
        public static long GATE_DURATION = 750;
        public static double GATE_HEADING = 160;
        public static double GATE_CYCLE_TM = 3000;

        // paths
        public static double ROW1_BRAKE_STRENGTH = 1.0;
        public static double PW_SCALE_GATE_CYCLE_SPEED = 0.7;
        public static double ROW2_INTAKE_PATH_SPEED = 0.7;

        // global path stuff
        public static double PW_SCALE_BRAKE_THRESHOLD = 0.87;
        public static double PW_SCALE_PATH_SPEED = 0.18;
        public static double PRELOAD_SLOWDOWN_THRESH = 0.5;

        private Robot robot;
        private PathChain shootPreload;
        private PathChain intakeRow1, intakeRow2;
        private PathChain scoreRow1, scoreRow2;
        private PathChain gateCycle, shootGate;
        private PathChain parkRP;

        public static int nRows = 4;

        @Override
        public void init() {
            Pose startPose = Robot.alliance == DuneStrider.Alliance.BLUE ? START_PRELOAD.setHeading(0) : START_PRELOAD.mirror().setHeading(heading(180));

            robot = Robot.get().init(DuneStrider.Mode.AUTO, startPose, hardwareMap, telemetry);
            robot.eyes.setEnabled(false);
            robot.turret.loadAngle(0);
            Turret.PREDICT_FACTOR = 0;

            Follower follower = robot.drive.follower;
            buildPathChains(follower);

            // we ball
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            execPreload(),
                            If(execRow2(), nothing(), () -> nRows >= 2),
                            If(execRowGate(), nothing(), () -> nRows >= 2),
                            If(execRowGate(), nothing(), () -> nRows >= 2),
                            If(execRow1(), nothing(), () -> nRows >= 1),
                            go(follower, parkRP, 1),
                            run(() -> robot.shooter.setVelocity(0))
                    )
            );
        }

        @Override
        public void init_loop() {
            telemetry.addLine("====DUNESTRIDER PRE-MATCH Config=====");
            telemetry.addData("|| > ALLIANCE:", Robot.alliance.toString());
            telemetry.addData("|| > ROWS:", nRows);
            telemetry.update();
        }

        @Override
        public void loop() {
            robot.endLoop();
        }

        private Command execPreload() {
            return new SequentialCommandGroup(
                    run(() -> robot.intake.openLatch()),
                    run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                    new FollowPathCommand(robot.drive.follower, shootPreload, true),
                    shoot((long) SHOOTER_TRANSFER_DELAY)
            );
        }

        private Command execRowGate() {
            return new SequentialCommandGroup(
                    // RUN INTAKE WITH ALIGN
                    run(() -> robot.intake.closeLatch()),
                    // eat the balls
                    intakeSet(Intake.Mode.INGEST),
                    new FollowPathCommand(robot.drive.follower, gateCycle, 1.0)
                            .raceWith(waitFor((long) GATE_CYCLE_TM)),
                    waitFor(GATE_DURATION),

                    // let the intake regen
                    fork (
                            new SequentialCommandGroup(
                                    waitFor((long) INTAKE_RECOLLECTION_TIMEOUT),
                                    intakeSet(Intake.Mode.OFF)
                            ),
                            new SequentialCommandGroup(
                                    run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                    new FollowPathCommand(robot.drive.follower, shootGate, true, 1.0)
                            )
                    ),

                    // go home and score
                    shoot((long) SHOOTER_TRANSFER_DELAY)
            );
        }

        private Command execRow1() {
            return new SequentialCommandGroup(
                    // RUN INTAKE WITH ALIGN
                    run(() -> robot.intake.closeLatch()),
                    // eat the balls
                    intakeSet(Intake.Mode.INGEST),
                    new FollowPathCommand(robot.drive.follower, intakeRow1, false, 1.0),

                    waitFor(INTAKE_STOP_DELAY),
                    // let the intake regen
                    fork (
                            new SequentialCommandGroup(
                                    waitFor((long) INTAKE_RECOLLECTION_TIMEOUT),
                                    intakeSet(Intake.Mode.OFF)
                            ),
                            new SequentialCommandGroup(
                                    run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                    new FollowPathCommand(robot.drive.follower, scoreRow1, true, 1.0)
                            )
                    ),

                    // go home and score
                    shoot((long) SHOOTER_TRANSFER_DELAY)
            );
        }

        private Command execRow2() {
            return new SequentialCommandGroup(
                    run(() -> robot.intake.closeLatch()),
                    // turn on the intake and eat up the balls
                    intakeSet(Intake.Mode.INGEST),
                    new FollowPathCommand(robot.drive.follower, intakeRow2, true, 1),
                    waitFor(INTAKE_STOP_DELAY),

                    fork(
                            new SequentialCommandGroup(
                                    waitFor((long) INTAKE_RECOLLECTION_TIMEOUT),
                                    intakeSet(Intake.Mode.OFF)
                            ),
                            new SequentialCommandGroup(
                                    run(() -> robot.shooter.setMode(Shooter.Mode.DYNAMIC)),
                                    new FollowPathCommand(robot.drive.follower, scoreRow2, true, 1)
                            )
                    ),

                    // go home and score
                    shoot((long) SHOOTER_TRANSFER_DELAY)
            );
        }

        private void buildPathChains(Follower follower) {
            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    mPBA(START_PRELOAD),
                                    mPBA(new Pose(49, 113)),
                                    mPBA(UNIVERSAL_SCORE_TARGET)
                            )
                    )
                    .addParametricCallback(PRELOAD_SLOWDOWN_THRESH, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                    .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                    .setTangentHeadingInterpolation()
                    .setBrakingStart(0.7)
                    .setBrakingStrength(ROW1_BRAKE_STRENGTH)
                    .build();

            intakeRow1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    mPBA(UNIVERSAL_SCORE_TARGET),
                                    mPBA(INTAKE_CONTROL_POINT),
                                    mPBA(END_INTAKE_START_SCORE)
                            )
                    )
                    .addParametricCallback(0, () -> follower.setMaxPowerScaling(ROW2_INTAKE_PATH_SPEED))
                    .addParametricCallback(PW_SCALE_BRAKE_THRESHOLD, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                    .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                    .setTangentHeadingInterpolation()
                    .build();

            scoreRow1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    mPBA(END_INTAKE_START_SCORE),
                                    mPBA(UNIVERSAL_SCORE_TARGET)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            mHBA(heading(180)),
                            mHBA(heading(-90))
                    )
                    .addParametricCallback(PW_SCALE_BRAKE_THRESHOLD, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                    .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                    .build();

            gateCycle = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    mPBA(UNIVERSAL_SCORE_TARGET),
                                    mPBA(new Pose(54, 55)),
                                    mPBA(INTAKE_GATE)
                            )
                    )
                    .addParametricCallback(0.1, () -> follower.setMaxPowerScaling(PW_SCALE_GATE_CYCLE_SPEED))
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    mPBA(INTAKE_GATE),
                                    mPBA(END_GATE)
                            )
                    )
                    .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                    .setConstantHeadingInterpolation(mHBA(heading(GATE_HEADING)))
                    .setTimeoutConstraint(100)
                    .build();

            shootGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    mPBA(END_GATE),
                                    mPBA(new Pose(52, 46)),
                                    mPBA(UNIVERSAL_SCORE_TARGET)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(PW_SCALE_BRAKE_THRESHOLD, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                    .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                    .build();

            intakeRow2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    mPBA(UNIVERSAL_SCORE_TARGET),
                                    mPBA(INTAKE_CONTROL_POINT2),
                                    mPBA(END_INTAKE_START_SCORE2)
                            )
                    )
                    .addParametricCallback(0.4, () -> follower.setMaxPowerScaling(ROW2_INTAKE_PATH_SPEED))
                    .setConstantHeadingInterpolation(mHBA(heading(180)))
                    .addParametricCallback(PW_SCALE_BRAKE_THRESHOLD, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                    .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                    .build();

            scoreRow2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    mPBA(END_INTAKE_START_SCORE2),
                                    mPBA(new Pose(52, 46)),
                                    mPBA(UNIVERSAL_SCORE_TARGET)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    /*
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0.0, 0.8, HeadingInterpolator.constant(mHBA(heading(180)))),
                                    new HeadingInterpolator.PiecewiseNode(0.3, 1, HeadingInterpolator.constant(mHBA(heading(-90))))
                            )
                    ) */
                    .addParametricCallback(0.5, () -> follower.setMaxPowerScaling(0.6))
                    .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                    .build();

            parkRP = follower.pathBuilder()
                    .addPath(new BezierLine(mPBA(UNIVERSAL_SCORE_TARGET), mPBA(new Pose(48, 72))))
                    .setTangentHeadingInterpolation()
                    .addParametricCallback(PW_SCALE_BRAKE_THRESHOLD, () -> follower.setMaxPowerScaling(PW_SCALE_PATH_SPEED))
                    .addParametricCallback(1, () -> follower.setMaxPowerScaling(1.0))
                    .build();
        }

        // Mirror Pose based on alliance
        public static Pose mPBA(Pose poseToMirror) {
            if (DuneStrider.alliance == DuneStrider.Alliance.RED) {
                return poseToMirror.mirror();
            } else {
                return poseToMirror;
            }
        }

        public static double mHBA(double heading) {
            if (DuneStrider.alliance == DuneStrider.Alliance.RED) {
                return mirrorHeading(heading);
            } else {
                return heading;
            }
        }
    }
