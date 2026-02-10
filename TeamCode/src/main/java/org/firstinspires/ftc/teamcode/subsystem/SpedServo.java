package org.firstinspires.ftc.teamcode.subsystem;

import static com.acmerobotics.roadrunner.Math.lerp;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.control.Ranges;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class SpedServo {

    public static double POSITION_TOLERANCE = 0.01;

    private final double t0, t1, halfRange, halfwayPt;
    final CachedSimpleServo servo;

    public SpedServo(CachedSimpleServo servo, double minDegree, double maxDegree) {
        this.servo = servo;
        this.t0 = minDegree;
        this.t1 = maxDegree;
//        halfRange = (t1 - t0) / 2;
//        halfwayPt = t0 + halfRange;
        halfRange = 0.5;
        halfwayPt = 0.5;
    }


    private double target;
    private boolean reachedGo, maxGo;

    public void setTarget(double extCurrent, double extTarget) {
        double current = lerp(extCurrent, t0, t1, 0.0, 1.0);
        this.target = lerp(extTarget, t0, t1, 0.0, 1.0);
        this.reachedGo = true;

        if (Math.abs(target - current) < halfRange) {
            servo.turnToAngle(Ranges.lerp(this.t0, this.t1, target));
            return;
        }

        // We need to wraparound
        reachedGo = false;
        maxGo = current > halfwayPt;
        servo.setPosition(maxGo ? 0 : 1);
    }

    public void run(double extCurrent) {

        double current = lerp(extCurrent, t0, t1, 0.0, 1.0);

        if (reachedGo)
            return;

        if (maxGo){
            if (current >= (0 + POSITION_TOLERANCE)) {
                reachedGo = true;
                servo.turnToAngle(Ranges.lerp(this.t0, this.t1, target));
            }
        } else {
            if (current <= (1 - POSITION_TOLERANCE)) {
                reachedGo = true;
                servo.turnToAngle(Ranges.lerp(this.t0, this.t1, target));
            }
        }
    }

}
