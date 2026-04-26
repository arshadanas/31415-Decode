package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Math.max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.Ranges;
import org.firstinspires.ftc.teamcode.opmode.Tele;

@Config
@TeleOp(group = "Testing/tuning")
public final class TuneFlywheelFeedforward extends LinearOpMode {

    private final ElapsedTime loopTimer = new ElapsedTime();
    public static double power = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        Flywheel flywheel = new Flywheel(hardwareMap);

        waitForStart(); // -------------------------------------------------------------------------------------------------------------------

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            flywheel.run(!gamepad1.square, !gamepad1.square);

            if (gamepad1.dpadUpWasPressed())
                power += 0.05;
            if (gamepad1.dpadDownWasPressed())
                power -= 0.05;

            power = Ranges.clip(power, 0.05, 1);

            double voltageScalar = Flywheel.MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
            flywheel.setManual(power * voltageScalar);

            Thread.sleep((long)(max(Tele.AVG_LOOP_TIME_MS - loopTimer.milliseconds(),0)));
            telemetry.addData("LOOP TIME", loopTimer.seconds());
            loopTimer.reset();
            telemetry.addData("Power", power);
            flywheel.printTo(telemetry);
            telemetry.update();
        }
    }
}
