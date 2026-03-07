package org.firstinspires.ftc.teamcode.control.controller;

import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.control.gainmatrix.FullStateGains;

public class FullStateController implements FeedbackController {

    private FullStateGains gains = new FullStateGains();
    private final State target = new State(), error = new State();

    @Override
    public void setTarget(State target) {
        this.target.set(target);
    }

    public void setGains(FullStateGains gains) {
        this.gains = gains;
    }

    @Override
    public double calculate(State measurement) {
        error.set(target).minusAssign(measurement);
        return gains.dot(error);
    }
}
