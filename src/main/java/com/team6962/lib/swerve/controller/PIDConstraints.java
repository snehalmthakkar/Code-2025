package com.team6962.lib.swerve.controller;

import edu.wpi.first.math.controller.PIDController;

public class PIDConstraints {
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double iZone = Double.POSITIVE_INFINITY;
    public double iMax = 1.0;
    public double iMin = -1.0;
    public double tolerance = 0.05;
    public double derivativeTolerance = Double.POSITIVE_INFINITY;
    public double period = 0.02;

    public PIDConstraints() {
    }

    public PIDConstraints(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDConstraints(double kP, double kI, double kD, double period) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.period = period;
    }

    public PIDConstraints withKP(double kP) {
        this.kP = kP;
        return this;
    }

    public PIDConstraints withKI(double kI) {
        this.kI = kI;
        return this;
    }

    public PIDConstraints withKD(double kD) {
        this.kD = kD;
        return this;
    }

    public PIDConstraints withIZone(double iZone) {
        this.iZone = iZone;
        return this;
    }

    public PIDConstraints withIMin(double iMin) {
        this.iMin = iMin;
        return this;
    }

    public PIDConstraints withIMax(double iMax) {
        this.iMax = iMax;
        return this;
    }

    public PIDConstraints withTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public PIDConstraints withDerivativeTolerance(double derivativeTolerance) {
        this.derivativeTolerance = derivativeTolerance;
        return this;
    }

    public PIDConstraints withPeriod(double period) {
        this.period = period;
        return this;
    }

    public PIDController createController() {
        var controller = new PIDController(kP, kI, kD, period);

        controller.setIZone(iZone);
        controller.setIntegratorRange(iMin, iMax);
        controller.setTolerance(tolerance, derivativeTolerance);

        return controller;
    }
}
