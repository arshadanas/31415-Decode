package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.Auto.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmode.Auto.pose;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.PURPLE;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystem.Motif;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.utility.Profiler;

import java.io.File;

import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

@Config
@TeleOp
public final class Tele extends LinearOpMode {

    private dev.nullftc.profiler.Profiler realProfiler;

    public static double AVG_LOOP_TIME_MS = 17.5;

    enum TeleOpConfig {
        EDITING_ALLIANCE,
        EDITING_SIDE;

        public static final TeleOpConfig[] selections = values();

        public TeleOpConfig plus(int i) {
            int max = selections.length;
            return selections[((ordinal() + i) % max + max) % max];
        }
        public String markIf(TeleOpConfig s) {
            return this == s ? "> " : "  ";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initProfiler();
        ElapsedTime matchTimer = new ElapsedTime();

        double TELE = 120; // seconds
        double LIFT_TIME = TELE - 15; // 15 seconds for lift

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, pose);
        robot.drivetrain.startTeleopDrive();

        TeleOpConfig selected = EDITING_ALLIANCE;

        boolean doTelemetry = false;
        boolean doProfiling = false;
        boolean isGoalSide = false;

        while (opModeInInit()) {

            if (gamepad1.dpadUpWasPressed()) {
                selected = selected.plus(-1);
            } else if (gamepad1.dpadDownWasPressed()) {
                selected = selected.plus(1);
            }

            if (gamepad1.triangle && gamepad1.square && gamepad1.dpad_left){
                doProfiling = !doProfiling;
            }

            switch (selected) {
                case EDITING_ALLIANCE:
                    if (gamepad1.squareWasPressed())
                        isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SIDE:
                    if (gamepad1.squareWasPressed())
                        isGoalSide = !isGoalSide;
            }

            robot.drivetrain.setHeadingWithStick(gamepad1.right_stick_x, gamepad1.right_stick_y, isRedAlliance);
            robot.drivetrain.update();

            telemetry.addLine();
            telemetry.addLine( EDITING_ALLIANCE.markIf(selected) + (isRedAlliance ? "RED" : "BLUE") + " alliance");
            telemetry.addLine();
            telemetry.addLine( EDITING_SIDE.markIf(selected) + "Starting in " + (isGoalSide ? "NEAR ZONE (Goal side)" : "FAR ZONE (Audience side)"));
            telemetry.addLine();
            telemetry.addData("Heading (deg, set with right stick)", toDegrees(robot.drivetrain.getPose().getHeading()));
            telemetry.addData("Will run profiler", doProfiling);

            telemetry.update();
        }

        Profiler.setProfiler(doProfiling ? realProfiler : null);

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------
        try {

            robot.setAlliance(isRedAlliance);
            pose = Auto.getStartingPose(isRedAlliance, isGoalSide);
            robot.drivetrain.setStartingPose(pose);
            robot.handler.container.preloadPGP();

            matchTimer.reset();

            // Control loop:
            while (opModeIsActive()) {
                Profiler.start("Main-robot-loop");

                Profiler.start("robot.run()");
                // Read sensors + gamepads:
                robot.run(gamepad1.square, gamepad1.dpad_down);

                Profiler.end("robot.run()");

                Profiler.start("update_controls");
                float triggersSum = gamepad1.right_trigger - gamepad1.left_trigger;

                if (gamepad1.left_bumper) {

                    if (gamepad1.squareWasPressed())
                        doTelemetry = !doTelemetry;

                    if (gamepad1.triangleWasPressed())
                        robot.lift.toggleHold();

                    if (gamepad1.crossWasPressed())
                        robot.lift.gearSwitch.toggle();

                    robot.drivetrain.setHeadingWithStick(gamepad1.right_stick_x, gamepad1.right_stick_y, isRedAlliance);

                    robot.lift.setPower(gamepad1.left_stick_y);

                    robot.handler.setFeederManual(gamepad1.left_trigger);
                    robot.shooter.setManual(gamepad1.right_trigger);

                } else {

                    // tracking classifier ramp (driver 2)
                    if (gamepad2.dpadUpWasPressed())
                        robot.handler.incrementArtifactsScored();
                    else if (gamepad2.dpadDownWasPressed())
                        robot.handler.decrementArtifactsScored();
                    else if (gamepad2.dpadLeftWasPressed() || gamepad2.dpadRightWasPressed())
                        robot.handler.clearRamp();

                    if (gamepad2.squareWasPressed())
                        robot.handler.randomization = Motif.GPP;
                    else if (gamepad2.triangleWasPressed())
                        robot.handler.randomization = Motif.PGP;
                    else if (gamepad2.circleWasPressed())
                        robot.handler.randomization = Motif.PPG;

                    if (gamepad1.triangleWasPressed())
                        robot.handler.container.preloadPGP();

                    if (gamepad1.crossWasPressed())
                        robot.handler.feedSingle(GREEN);

                    if (gamepad1.circleWasPressed())
                        robot.handler.feedSingle(PURPLE);

                    if (gamepad1.dpadRightWasPressed())
                        robot.handler.feedFastest();

                    if (gamepad1.dpadUpWasPressed())
                        robot.handler.feedMotif();

                    if (gamepad1.dpadLeftWasPressed())
                        robot.handler.motifMode = !robot.handler.motifMode;


                    robot.handler.setIntake(triggersSum);

                    robot.drivetrain.run(
                            gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x,
                            gamepad1.right_bumper /*|| triggersSum > 0 */,
                            isRedAlliance
                    );

                }
                Profiler.end("update_controls");

                Profiler.start("update_telemetry");
                if (doTelemetry) {
                    robot.printTo(telemetry);
                    telemetry.update();
                }
                Profiler.end("update_telemetry");

                Profiler.end("Main-robot-loop");
            }
        } finally {
            exportProfiler();
            Profiler.setProfiler(null);
            telemetry.update();
        }
    }



    private File profilerOutput;

    private void initProfiler(){
        // Create profile log folder
        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logsFolder.exists())
            logsFolder.mkdirs();

        long timestamp = System.currentTimeMillis();
        profilerOutput = new File(logsFolder, "profiler-" + timestamp + ".csv");

        realProfiler = dev.nullftc.profiler.Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(profilerOutput))
                .build();
    }

    /**
     * Exporting is computationally expensive, and has been optimized to the best it can be,
     * but to be safe we create a new thread to export on as to not have the program be stuck in stop()
     */
    private void exportProfiler() {
        // Check if we are actually in profile mode
        if (!Profiler.enabled)
            return;

        RobotLog.i("Starting async profiler export to: " + profilerOutput.getAbsolutePath());

        Thread exportThread = new Thread(() -> {
            try {
                realProfiler.export();
                realProfiler.shutdown();
            } catch (Exception e) {
                e.printStackTrace();
            }
        });

        exportThread.setDaemon(true);
        exportThread.start();
    }
}
