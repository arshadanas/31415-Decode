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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@TeleOp
public final class Tele extends LinearOpMode {

    enum TeleOpConfig {
        EDITING_ALLIANCE;

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

        ElapsedTime matchTimer = new ElapsedTime();

        double TELE = 120; // seconds
        double LIFT_TIME = TELE - 15; // 15 seconds for lift

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, pose);
        robot.drivetrain.startTeleopDrive();

        TeleOpConfig selected = EDITING_ALLIANCE;

        boolean doTelemetry = false;

        while (opModeInInit()) {

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
        }

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------

        robot.setAlliance(isRedAlliance);

        matchTimer.reset();

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.run();

            float triggersSum = gamepad1.right_trigger - gamepad1.left_trigger;

            if (gamepad1.left_bumper) {

                if (gamepad1.squareWasPressed()) doTelemetry = !doTelemetry;

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


                if (gamepad1.triangleWasPressed())
                    robot.lift.gearSwitch.toggle();

                if (gamepad1.squareWasPressed())
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

            if (doTelemetry) {
                robot.printTo(telemetry);
                telemetry.update();
            }

//            double t = matchTimer.seconds();
//            indicator.setColor(t >= LIFT_TIME ?
//                    sin(PI * t / TIME_CLIMB_INDICATOR_ON) >= 0 ? GREEN : OFF :
//                    robot.intake.hasSample() ? GREEN :
//                            OFF
//            );
        }
    }
}
