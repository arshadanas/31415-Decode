package org.firstinspires.ftc.teamcode.control.controller;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import org.firstinspires.ftc.teamcode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.control.motion.Integrator;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.control.filter.Filter;
import org.firstinspires.ftc.teamcode.control.filter.NoFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.PIDGains;

public class PIDController implements FeedbackController {

    private PIDGains gains = new PIDGains();
    private final State target = new State(), error = new State();

    private final Filter derivFilter;
    private final Differentiator differentiator = new Differentiator();
    private final Differentiator filterDiff = new Differentiator();
    private final Integrator integrator = new Integrator();

    private double errorIntegral, filteredErrorDerivative, rawErrorDerivative;

    public PIDController() {
        this(new NoFilter());
    }

    public PIDController(Filter derivFilter) {
        this.derivFilter = derivFilter;
    }

    public void setGains(PIDGains gains) {
        this.gains = gains;
    }

    /**
     * @param measurement The x attribute of the given {@link State} is used to calculate error for feedback.
     *                    If the v attribute is non-zero, -v will be used as the {@link #filteredErrorDerivative}
     */
    public double calculate(State measurement) {
        double lastError = error.x;
        error.set(target).minusAssign(measurement);

        if (signum(error.x) != signum(lastError)) reset();
        errorIntegral = integrator.getIntegral(error.x);
        rawErrorDerivative = differentiator.getDerivative(error.x);
        filteredErrorDerivative = filterDiff.getDerivative(derivFilter.calculate(error.x));
        if (measurement.v != 0)
            filteredErrorDerivative = -measurement.v;

        double output = (gains.kP * error.x) + (gains.kI * errorIntegral) + (gains.kD * filteredErrorDerivative);

        stopIntegration(abs(output) >= gains.maxOutputWithIntegral && signum(output) == signum(error.x));

        return output;
    }

    public void setTarget(State target) {
        this.target.set(target);
    }

    public double getFilteredErrorDerivative() {
        return filteredErrorDerivative;
    }

    public double getRawErrorDerivative() {
        return rawErrorDerivative;
    }

    public double getErrorIntegral() {
        return errorIntegral;
    }

    public void stopIntegration(boolean stopIntegration) {
        integrator.stopIntegration(stopIntegration);
    }

    public void reset() {
        integrator.reset();
        derivFilter.reset();
    }
}
