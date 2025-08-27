package com.team6962.lib.swerve.controller;

public class TrapezoidalConstraints {
    private final double maxVelocity;
    private final double maxAcceleration;

    public TrapezoidalConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    @Override
    public String toString() {
        return "Trapezoidal{maxVelocity=" + maxVelocity + ", maxAcceleration=" + maxAcceleration + "}";
    }

    public static double computeTrapezoidalProfileTime(TrapezoidalConstraints constraints, double distance) {
        double minimumTrapezoidalProfileDistance = constraints.maxVelocity / constraints.maxAcceleration;

        if (distance >= minimumTrapezoidalProfileDistance) {
            return constraints.maxVelocity / constraints.maxAcceleration + distance / constraints.maxVelocity;
        } else {
            return 2 * Math.sqrt(distance / constraints.maxAcceleration);
        }
    }

    public static record TrapezoidProfileAndDistance(TrapezoidalConstraints constraints, double distance) {
        @Override
        public final String toString() {
            return "TrapezoidProfileAndDistance{" +
                "constraints=" + constraints +
                ", distance=" + distance +
                '}';
        }
    }

    public static TrapezoidalConstraints scaleTrapezoidalConstraintsToTime(TrapezoidalConstraints constraints, double time, double distance) {
        double trapezoidalMaxVelocity = distance / (time - constraints.maxVelocity / constraints.maxAcceleration);
        double trapezoidalMaxAcceleration = constraints.maxAcceleration / constraints.maxVelocity * trapezoidalMaxVelocity;

        double minimumTrapezoidalProfileDistance = trapezoidalMaxVelocity / trapezoidalMaxAcceleration;

        if (distance >= minimumTrapezoidalProfileDistance) {
            return new TrapezoidalConstraints(trapezoidalMaxVelocity, trapezoidalMaxAcceleration);
        }

        double triangularMaxAcceleration = distance / Math.pow(time / 2, 2);
        double triangularMaxVelocity = constraints.maxVelocity / constraints.maxAcceleration * triangularMaxAcceleration;

        return new TrapezoidalConstraints(triangularMaxVelocity, triangularMaxAcceleration);
    }

    // TODO: Support initial and exit velocities
    public static TrapezoidalConstraints[] matchProfileDurations(TrapezoidProfileAndDistance ...profiles) {
        double[] times = new double[profiles.length];
        double targetTime = 0.0;

        for (int i = 0; i < profiles.length; i++) {
            TrapezoidProfileAndDistance profile = profiles[i];

            times[i] = computeTrapezoidalProfileTime(profile.constraints, profile.distance);

            if (targetTime < times[i]) {
                targetTime = times[i];
            }
        }

        TrapezoidalConstraints[] matchedProfiles = new TrapezoidalConstraints[profiles.length];

        for (int i = 0; i < profiles.length; i++) {
            double currentTime = times[i];

            if (currentTime < 1e-6) {
                matchedProfiles[i] = profiles[i].constraints;
                continue;
            }

            matchedProfiles[i] = scaleTrapezoidalConstraintsToTime(profiles[i].constraints, targetTime, profiles[i].distance);
        }

        return matchedProfiles;
    }
}
