package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.Auto.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmode.Auto.pose;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_ALLIANCE;
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

                if (gamepad1.crossWasPressed())
                    robot.lift.enabled = !robot.lift.enabled;
                robot.lift.setManualPower(gamepad1.left_stick_y);

                robot.handler.runFeeder(triggersSum);

            } else {

                robot.handler.runIntake(triggersSum);

                if (!robot.lift.enabled)
                    robot.drivetrain.run(
                            gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x,
                            gamepad1.right_bumper /*|| triggersSum > 0 */,
                            isRedAlliance
                    );

            }

            if (doTelemetry) {
                robot.print(telemetry);
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
