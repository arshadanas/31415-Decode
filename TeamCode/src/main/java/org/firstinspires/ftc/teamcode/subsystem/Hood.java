package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.control.Ranges.clip;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.Ranges;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Hood {

    public static double
            LAUNCH_RAD_SHALLOWEST = 0.5567832093586575, // 31.901328 deg
            LAUNCH_RAD_STEEPEST = 1.0776000610289713, // 61.7419355 deg

            ANGLE_HOOD_SHALLOWEST = 360,
            ANGLE_HOOD_STEEPEST = 10,

            CACHE_THRESHOLD_HOOD = 0.05;

    private final CachedSimpleServo hood;

    public Hood(HardwareMap hardwareMap) {
        hood = new CachedSimpleServo(hardwareMap, "hood", 0, 360).reversed();
    }

    /**
     * @param radians Launch angle (in radians, where 0 is horizontal, parallel to the floor)
     *                in the range [{@link #LAUNCH_RAD_SHALLOWEST}, {@link #LAUNCH_RAD_STEEPEST}]
     */
    public void setLaunchAngle(double radians) {
        hood.threshold = CACHE_THRESHOLD_HOOD; // TODO adjust when hood angle becomes continuous and differentiable
        hood.turnToAngle(Ranges.lerp(
                clip(radians, LAUNCH_RAD_SHALLOWEST, LAUNCH_RAD_STEEPEST),
                LAUNCH_RAD_SHALLOWEST, LAUNCH_RAD_STEEPEST, // TODO Tune empirically
                ANGLE_HOOD_SHALLOWEST, ANGLE_HOOD_STEEPEST
        ));
    }


}
