package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.Auto.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmode.Auto.sharedArtifacts;
import static org.firstinspires.ftc.teamcode.opmode.Auto.sharedPose;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleConfig.EDITING_HEADING;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleConfig.EDITING_PRELOAD;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleConfig.EDITING_SIDE;
import static java.lang.Math.PI;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Handler;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Config
@TeleOp
public final class Tele extends LinearOpMode {

    public static double AVG_LOOP_TIME_MS = 17.5;

    enum TeleConfig {
        EDITING_ALLIANCE,
        EDITING_SIDE,
        EDITING_PRELOAD,
        EDITING_HEADING;

        public static final TeleConfig[] selections = values();

        public TeleConfig plus(int i) {
            int max = selections.length;
            return selections[((ordinal() + i) % max + max) % max];
        }
        public String markIf(TeleConfig s) {
            return this == s ? "> " : "  ";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, new Pose());
        robot.drivetrain.startTeleopDrive();

        Pose wallResetPose = new Pose(
                Auto.WIDTH_DRIVETRAIN / 2.0,
                Auto.LENGTH_DRIVETRAIN / 2.0,
                PI / 2
        );

        robot.handler.setContents(sharedArtifacts);
        // expire the shared artifacts
        sharedArtifacts[0] = sharedArtifacts[1] = sharedArtifacts[2] = false;

        TeleConfig selected = EDITING_ALLIANCE;

        boolean doTelemetry = false;
        boolean isGoalSide = false;

        while (opModeInInit()) {

            if (gamepad1.dpadUpWasPressed())
                selected = selected.plus(-1);
            else if (gamepad1.dpadDownWasPressed())
                selected = selected.plus(1);

            if (gamepad1.squareWasPressed()) switch (selected) {
                case EDITING_ALLIANCE:
                    isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SIDE:
                    isGoalSide = !isGoalSide;
                    break;
                case EDITING_PRELOAD:
                    robot.handler.setContents(robot.handler.hasArtifacts() ? Handler.EMPTY : Handler.FULL);
                    break;
            }

            if (selected == EDITING_HEADING)
                robot.drivetrain.setHeadingWithStick(gamepad1.right_stick_x, gamepad1.right_stick_y, isRedAlliance);
            robot.drivetrain.update();

            telemetry.addLine();
            telemetry.addLine(EDITING_ALLIANCE.markIf(selected) + (isRedAlliance ? "RED" : "BLUE") + " alliance");
            telemetry.addLine();
            telemetry.addLine(EDITING_SIDE.markIf(selected) + "Starting in " + (isGoalSide ? "near zone (GOAL SIDE)" : "far zone (AUDIENCE SIDE)"));
            telemetry.addLine();
            telemetry.addData(EDITING_PRELOAD.markIf(selected) + "Preloaded", robot.handler.getContents());
            telemetry.addLine();
            telemetry.addLine(EDITING_HEADING.markIf(selected) + "Heading (deg), set with right stick:");
            telemetry.addLine("  " + toDegrees(robot.drivetrain.getPose().getHeading()));
            telemetry.update();
        }

        // ----------------------------------------------------------------------------------------------------------------------------------------------------------------

        robot.setAlliance(isRedAlliance);
        robot.drivetrain.setPose(sharedPose != null ? sharedPose : Auto.getStartingPose(isRedAlliance, isGoalSide));

        if (!isRedAlliance) wallResetPose = wallResetPose.mirror();

        // Control loop:
        while (opModeIsActive()) {

            // Read sensors + gamepads:
            robot.run(gamepad1.square, gamepad1.dpad_down);

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

                if (gamepad1.dpadRightWasPressed())
                    robot.drivetrain.setPose(wallResetPose);
                else if (gamepad1.dpadDownWasPressed() && sharedPose != null)
                    robot.drivetrain.setPose(sharedPose);

            } else {

                robot.handler.setIntake(triggersSum);

                robot.drivetrain.run(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.right_bumper,
                        isRedAlliance
                );

            }

            if (doTelemetry) {
                robot.printTo(telemetry);
                telemetry.update();
            }
        }

        sharedPose = null; // expire the shared pose
    }
}
