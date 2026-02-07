package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.Auto.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmode.Auto.pose;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_SIDE;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.utility.Profiler;

import java.io.File;

import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

@Config
@Autonomous(preselectTeleOp = "Tele")
public final class AutoSped extends LinearOpMode {

    public static double MOVE_POWER = 0.5, TIME_MOVE_NEAR = 0.5, TIME_MOVE_FAR = 0.5;

    private dev.nullftc.profiler.Profiler realProfiler;

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

        Tele.TeleOpConfig selected = EDITING_ALLIANCE;

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

        Profiler.setProfiler(null);

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------
        try {

            robot.setAlliance(isRedAlliance);
            pose = Auto.getStartingPose(isRedAlliance, isGoalSide);
            robot.drivetrain.setStartingPose(pose);
//            robot.handler.container.preloadPGP();

            double timeMoving = isGoalSide ? TIME_MOVE_NEAR : TIME_MOVE_FAR;

            matchTimer.reset();

            // Control loop:
            while (opModeIsActive() && matchTimer.seconds() <= timeMoving) {
                Profiler.start("Main-robot-loop");

                Profiler.start("robot.run()");
                // Read sensors + gamepads:
                robot.run(false, false);

                Profiler.end("robot.run()");

                if (isGoalSide)
                    robot.drivetrain.run(
                            (isRedAlliance ? 1 : -1) * MOVE_POWER,
                            0,
                            0,
                            false /*|| triggersSum > 0 */,
                            isRedAlliance
                    );
                else
                    robot.drivetrain.run(
                            0,
                            -MOVE_POWER,
                            0,
                            false /*|| triggersSum > 0 */,
                            isRedAlliance
                    );

                pose = robot.drivetrain.getPose();

//                Profiler.start("update_telemetry");
//                if (doTelemetry) {
//                    robot.printTo(telemetry);
//                    telemetry.update();
//                }
//                Profiler.end("update_telemetry");

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
