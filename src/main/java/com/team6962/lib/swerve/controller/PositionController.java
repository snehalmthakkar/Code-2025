package com.team6962.lib.swerve.controller;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PositionController implements AutoCloseable {
    private TrapezoidProfile motionProfile;
    private PIDController pidController;
    private boolean finished;

    private double lastTime, pidOutput, profilePosition, profileVelocity;
    private TrapezoidProfile.State lastStart, lastEnd, lastCurrent;

    private Runnable closeLogging;

    public PositionController(TrapezoidProfile.Constraints profileConstraints, PIDConstraints pidConstraints) {
        this.motionProfile = new TrapezoidProfile(profileConstraints);
        this.pidController = pidConstraints.createController();
    }

    public PositionController withLogging(String name) {
        closeLogging = Logger.addUpdate(name, () -> {
            if (lastStart == null || lastEnd == null || lastCurrent == null) return;

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
            Logger.log(name + "/isFinished", isFinished());
        });

        return this;
    }

    public double calculateVelocity(double time, TrapezoidProfile.State start, TrapezoidProfile.State end, TrapezoidProfile.State current) {
        this.lastTime = time;
        this.lastStart = start;
        this.lastEnd = end;
        this.lastCurrent = current;

        TrapezoidProfile.State target = motionProfile.calculate(time, start, end);
        double pidOutput = pidController.calculate(current.position, target.position);

        this.pidOutput = pidOutput;
        this.profileVelocity = target.velocity;
        
        profilePosition = target.position;

        finished = /*Math.abs(target.position - end.position) < tolerance && */motionProfile.isFinished(time);

        return target.velocity + pidOutput;
    }

    public boolean isFinished() {
        return finished;
    }

    @Override
    public void close() {
        if (closeLogging != null) {
            closeLogging.run();
        }
    }
}
