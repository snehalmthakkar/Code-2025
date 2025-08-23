package com.team6962.lib.swerve.controller;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PositionController implements AutoCloseable {
    public static class ProfileState {
        public double position;
        public double velocity;

        public ProfileState(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        public ProfileState(TrapezoidProfile.State state) {
            this.position = state.position;
            this.velocity = state.velocity;
        }

        public ProfileState(ExponentialProfile.State state) {
            this.position = state.position;
            this.velocity = state.velocity;
        }

        public TrapezoidProfile.State toTrapezoidal() {
            return new TrapezoidProfile.State(position, velocity);
        }

        public ExponentialProfile.State toExponential() {
            return new ExponentialProfile.State(position, velocity);
        }
    }

    public static interface MotionProfile {
        public ProfileState calculate(double time, ProfileState start, ProfileState end);
        public boolean isFinished(double time);
    }

    public static class TrapezoidalMotionProfile implements MotionProfile {
        private TrapezoidProfile profile;

        public TrapezoidalMotionProfile(TrapezoidProfile.Constraints constraints) {
            this.profile = new TrapezoidProfile(constraints);
        }

        @Override
        public ProfileState calculate(double time, ProfileState start, ProfileState end) {
            return new ProfileState(profile.calculate(time, start.toTrapezoidal(), end.toTrapezoidal()));
        }

        @Override
        public boolean isFinished(double time) {
            return profile.isFinished(time);
        }
    }

    public static class ExponentialMotionProfile implements MotionProfile {
        private ExponentialProfile profile;
        private double totalTime;

        public ExponentialMotionProfile(ExponentialProfile.Constraints constraints) {
            this.profile = new ExponentialProfile(constraints);
        }

        @Override
        public ProfileState calculate(double time, ProfileState start, ProfileState end) {
            totalTime = profile.timeLeftUntil(start.toExponential(), end.toExponential());

            return new ProfileState(profile.calculate(time, start.toExponential(), end.toExponential()));
        }

        @Override
        public boolean isFinished(double time) {
            return time >= totalTime;
        }
    }

    private MotionProfile motionProfile;
    private PIDController pidController;

    private double lastTime, pidOutput, profilePosition, profileVelocity;
    private ProfileState lastStart, lastEnd, lastCurrent;

    private Runnable closeLogging;

    public PositionController(MotionProfile motionProfile, PIDController pidController) {
        this.motionProfile = motionProfile;
        this.pidController = pidController;
    }

    public PositionController withLogging(String name) {
        closeLogging = Logger.addUpdate(name, () -> {
            Logger.log(name + "/timeSeconds", lastTime);
            Logger.log(name + "/start/position", lastStart.position);
            Logger.log(name + "/start/velocity", lastStart.velocity);
            Logger.log(name + "/end/position", lastEnd.position);
            Logger.log(name + "/end/velocity", lastEnd.velocity);
            Logger.log(name + "/current/position", lastCurrent.position);
            Logger.log(name + "/current/velocity", lastCurrent.velocity);
            Logger.log(name + "/profile/position", profilePosition);
            Logger.log(name + "/profile/velocity", profileVelocity);
            Logger.log(name + "/velocityOutput/pid", pidOutput);
            Logger.log(name + "/velocityOutput/profile", profileVelocity);
            Logger.log(name + "/velocityOutput/total", profileVelocity + pidOutput);
            Logger.log(name + "/isFinished", isFinished(lastTime));
        });

        return this;
    }

    public double calculateVelocity(double time, ProfileState start, ProfileState end, ProfileState current) {
        this.lastTime = time;
        this.lastStart = start;
        this.lastEnd = end;
        this.lastCurrent = current;

        ProfileState target = motionProfile.calculate(time, start, end);
        double pidOutput = pidController.calculate(current.position, target.position);

        this.pidOutput = pidOutput;
        this.profileVelocity = target.velocity;
        
        profilePosition = target.position;

        return target.velocity + pidOutput;
    }

    public boolean isFinished(double time) {
        return motionProfile.isFinished(time);
    }

    @Override
    public void close() {
        if (closeLogging != null) {
            closeLogging.run();
        }
    }
}
