package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.Auto.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmode.Auto.pose;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.subsystem.Artifact.PURPLE;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

import java.io.File;

import dev.nullftc.profiler.Profiler;
import dev.nullftc.profiler.entry.BasicProfilerEntryFactory;
import dev.nullftc.profiler.exporter.CSVProfilerExporter;

@TeleOp(name = "Tele With Profiling")
public final class TeleWithProfiling extends LinearOpMode {


    private Profiler profiler;

    @Override
    public void runOpMode() throws InterruptedException {
        File logsFolder = new File(AppUtil.FIRST_FOLDER, "logs");
        if (!logsFolder.exists()) logsFolder.mkdirs();

        long timestamp = System.currentTimeMillis();
        File file = new File(logsFolder, "profiler-" + timestamp + ".csv");

        profiler = Profiler.builder()
                .factory(new BasicProfilerEntryFactory())
                .exporter(new CSVProfilerExporter(file))
                .debugLog(false)
                .build();

        try {
            profiler.start("Init");


            ElapsedTime matchTimer = new ElapsedTime();

            double TELE = 120; // seconds
            double LIFT_TIME = TELE - 15; // 15 seconds for lift

            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            Robot robot = new Robot(hardwareMap, pose, profiler);
            robot.drivetrain.startTeleopDrive();

            Tele.TeleOpConfig selected = EDITING_ALLIANCE;

            boolean doTelemetry = false;

            profiler.end("Init");


            while (opModeInInit()) {

                profiler.start("Init-loop");
                if (gamepad1.dpadUpWasPressed()) {
                    selected = selected.plus(-1);
                } else if (gamepad1.dpadDownWasPressed()) {
                    selected = selected.plus(1);
                }

                switch (selected) {
                    case EDITING_ALLIANCE:
                        if (gamepad1.squareWasPressed())
                            isRedAlliance = !isRedAlliance;
                        break;
                }

                robot.drivetrain.setHeadingWithStick(gamepad1.right_stick_x, gamepad1.right_stick_y, isRedAlliance);
                robot.drivetrain.update();

                telemetry.addLine();
                telemetry.addLine( EDITING_ALLIANCE.markIf(selected) + (isRedAlliance ? "RED" : "BLUE") + " alliance");
                telemetry.addLine();
                telemetry.addData("Heading (deg, set with right stick)", toDegrees(robot.drivetrain.getPose().getHeading()));

                telemetry.update();
                profiler.end("Init-loop");

            }

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------

            robot.setAlliance(isRedAlliance);

            matchTimer.reset();

            // Control loop:
            while (opModeIsActive()) {
                profiler.start("Main-robot-loop");
                // Read sensors + gamepads:

                profiler.start("robot.run()");
                robot.run(gamepad1.square);
                profiler.end("robot.run()");

                float triggersSum = gamepad1.right_trigger - gamepad1.left_trigger;

                if (gamepad1.left_bumper) {

                    if (gamepad1.squareWasPressed()) doTelemetry = !doTelemetry;

                    robot.drivetrain.setHeadingWithStick(gamepad1.right_stick_x, gamepad1.right_stick_y, isRedAlliance);

                    robot.lift.setPower(gamepad1.left_stick_y);

                    robot.handler.setFeederManual(gamepad1.left_trigger);
                    robot.shooter.setManual(gamepad1.right_trigger);

                } else {


                    profiler.start("dpad stuff");
                    // tracking classifier ramp (driver 2)
                    if (gamepad2.dpadUpWasPressed())
                        robot.handler.incrementArtifactsScored();
                    else if (gamepad2.dpadDownWasPressed())
                        robot.handler.decrementArtifactsScored();
                    else if (gamepad2.dpadLeftWasPressed() || gamepad2.dpadRightWasPressed())
                        robot.handler.clearRamp();

                    profiler.end("dpad stuff");

                    profiler.start("other buttons");
                    if (gamepad1.triangleWasPressed())
                        robot.lift.gearSwitch.toggle();

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

                    profiler.end("other buttons");

                    profiler.start("run drivetrain");
                    robot.drivetrain.run(
                            gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x,
                            gamepad1.right_bumper /*|| triggersSum > 0 */,
                            isRedAlliance
                    );
                    profiler.end("run drivetrain");

                }
                profiler.start("Telemetry_print");

                if (doTelemetry) {
                    robot.printTo(telemetry);
                    telemetry.update();
                }

                profiler.end("Telemetry_print");

                profiler.end("Main-robot-loop");
                //            double t = matchTimer.seconds();
                //            indicator.setColor(t >= LIFT_TIME ?
                //                    sin(PI * t / TIME_CLIMB_INDICATOR_ON) >= 0 ? GREEN : OFF :
                //                    robot.intake.hasSample() ? GREEN :
                //                            OFF
                //            );

            }

        } finally {
                exportProfiler(file);
                telemetry.update();
            }
        }


    /**
     * Exporting is computationally expensive, and has been optimized to the best it can be,
     * but to be safe we create a new thread to export on as to not have the program be stuck in stop()
     */
    private void exportProfiler(File file) {
        RobotLog.i("Starting async profiler export to: " + file.getAbsolutePath());

        Thread exportThread = new Thread(() -> {
            try {
                profiler.export();
                profiler.shutdown();
            } catch (Exception e) {
                e.printStackTrace();
            }
        });

        exportThread.setDaemon(true);
        exportThread.start();
    }

}
