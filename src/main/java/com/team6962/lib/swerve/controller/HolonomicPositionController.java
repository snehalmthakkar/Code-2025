package com.team6962.lib.swerve.controller;

import static edu.wpi.first.units.Units.Radians;

import com.team6962.lib.swerve.controller.PositionController.ProfileState;
import com.team6962.lib.swerve.controller.TrapezoidalConstraints.TrapezoidProfileAndDistance;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;

public class HolonomicPositionController implements AutoCloseable {
    private TrapezoidalConstraints linearProfile, angularProfile;
    private PIDConstraints linearFeedback, angularFeedback;

    private State previousStart;
    private State previousEnd;

    private PositionController xController;
    private PositionController yController;
    private PositionController thetaController;

    public HolonomicPositionController(
        TrapezoidalConstraints linearProfile,
        PIDConstraints linearFeedback,
        TrapezoidalConstraints angularProfile,
        PIDConstraints angularFeedback
    ) {
        this.linearProfile = linearProfile;
        this.angularProfile = angularProfile;
        this.linearFeedback = linearFeedback;
        this.angularFeedback = angularFeedback;
    }

    @SuppressWarnings("resource")
    private void updateProfiles(TrapezoidalConstraints xProfile, TrapezoidalConstraints yProfile, TrapezoidalConstraints angularProfile) {
        if (this.xController != null) {
            this.xController.close();
        }

        if (this.yController != null) {
            this.yController.close();
        }

        if (this.thetaController != null) {
            this.thetaController.close();
        }

        this.xController = new PositionController(xProfile.createProfile(), linearFeedback.createController()).withLogging("HolonomicPositionController/XMeters");
        this.yController = new PositionController(yProfile.createProfile(), linearFeedback.createController()).withLogging("HolonomicPositionController/YMeters");
        this.thetaController = new PositionController(angularProfile.createProfile(), angularFeedback.createController()).withLogging("HolonomicPositionController/ThetaRadians");
    }

    public static class State {
        public Pose2d position;
        public ChassisSpeeds velocity;

        public State(Pose2d position, ChassisSpeeds velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        public ProfileState getXState() {
            return new ProfileState(position.getX(), velocity.vxMetersPerSecond);
        }

        public ProfileState getYState() {
            return new ProfileState(position.getY(), velocity.vyMetersPerSecond);
        }

        public ProfileState getThetaState() {
            return new ProfileState(position.getRotation().getRadians(), velocity.omegaRadiansPerSecond);
        }

        public boolean equals(State other) {
            return this.position.equals(other.position) &&
                Math.abs(this.velocity.vxMetersPerSecond - other.velocity.vxMetersPerSecond) < 1e-6 &&
                Math.abs(this.velocity.vyMetersPerSecond - other.velocity.vyMetersPerSecond) < 1e-6 &&
                Math.abs(this.velocity.omegaRadiansPerSecond - other.velocity.omegaRadiansPerSecond) < 1e-6;
        }
    }

    public ChassisSpeeds calculatePathCoordinates(double time, State start, State end, State current) {
        double startX = start.position.getX();
        double startY = start.position.getY();

        double endX = end.position.getX();
        double endY = end.position.getY();

        double displacementX = endX - startX;
        double displacementY = endY - startY;

        double pathLength = Math.hypot(displacementX, displacementY);

        double mainX = displacementX / pathLength;
        double mainY = displacementY / pathLength;

        double crossX = -mainY;
        double crossY = mainX;

        double currentX = current.position.getX() - startX;
        double currentY = current.position.getY() - startY;

        double currentPosMain = mainX * currentX + mainY * currentY;
        double currentPosCross = crossX * currentX + crossY * currentY;

        double currentVelMain = current.velocity.vxMetersPerSecond * mainX + current.velocity.vyMetersPerSecond * mainY;
        double currentVelCross = current.velocity.vxMetersPerSecond * crossX + current.velocity.vyMetersPerSecond * crossY;

        double startVelMain = start.velocity.vxMetersPerSecond * mainX + start.velocity.vyMetersPerSecond * mainY;
        double startVelCross = start.velocity.vxMetersPerSecond * crossX + start.velocity.vyMetersPerSecond * crossY;

        double endVelMain = end.velocity.vxMetersPerSecond * mainX + end.velocity.vyMetersPerSecond * mainY;
        double endVelCross = end.velocity.vxMetersPerSecond * crossX + end.velocity.vyMetersPerSecond * crossY;

        ChassisSpeeds pathRelativeSpeeds = calculateFieldCoordinates(
            time,
            new State(
                new Pose2d(0, 0, start.position.getRotation()),
                new ChassisSpeeds(startVelMain, startVelCross, start.velocity.omegaRadiansPerSecond)
            ),
            new State(
                new Pose2d(0, pathLength, end.position.getRotation()),
                new ChassisSpeeds(endVelMain, endVelCross, end.velocity.omegaRadiansPerSecond)
            ),
            new State(
                new Pose2d(currentPosMain, currentPosCross, current.position.getRotation()),
                new ChassisSpeeds(currentVelMain, currentVelCross, current.velocity.omegaRadiansPerSecond)
            )
        );

        double targetVelMain = pathRelativeSpeeds.vxMetersPerSecond;
        double targetVelCross = pathRelativeSpeeds.vyMetersPerSecond;

        double targetVelX = mainX * targetVelMain + crossX * targetVelCross;
        double targetVelY = mainY * targetVelMain + crossY * targetVelCross;

        return new ChassisSpeeds(
            targetVelX,
            targetVelY,
            pathRelativeSpeeds.omegaRadiansPerSecond
        );
    }

    /**
     * Not generally recommended, as the speeds are more likely to become oversaturated than the path coordinates.
     */
    public ChassisSpeeds calculateFieldCoordinates(double time, State start, State end, State current) {
        if (previousStart == null || previousEnd == null || !previousStart.equals(start) || !previousEnd.equals(end)) {
            previousStart = start;
            previousEnd = end;

            System.out.println("New profile!");

            TrapezoidalConstraints[] constraints = TrapezoidalConstraints.matchProfileDurations(
                new TrapezoidProfileAndDistance(linearProfile, Math.abs(start.position.getX() - end.position.getX())),
                new TrapezoidProfileAndDistance(linearProfile, Math.abs(start.position.getY() - end.position.getY())),
                new TrapezoidProfileAndDistance(angularProfile, MeasureMath.minAbsDifference(
                    start.position.getRotation().getMeasure(),
                    end.position.getRotation().getMeasure()
                ).in(Radians))
            );

            System.out.println("X: " + constraints[0]);
            System.out.println("Y: " + constraints[1]);
            System.out.println("Theta: " + constraints[2]);

            updateProfiles(constraints[0], constraints[1], constraints[2]);
        }

        Angle currentAngle = current.position.getRotation().getMeasure();
        Angle endAngle = end.position.getRotation().getMeasure();

        Angle optimizedEndAngle = currentAngle.plus(MeasureMath.minDifference(endAngle, currentAngle));

        ProfileState optimizedAngularEndState = new ProfileState(
            optimizedEndAngle.in(Radians),
            end.velocity.omegaRadiansPerSecond
        );

        return new ChassisSpeeds(
            xController.calculateVelocity(time, start.getXState(), end.getXState(), current.getXState()),
            yController.calculateVelocity(time, start.getYState(), end.getYState(), current.getYState()),
            thetaController.calculateVelocity(time, start.getThetaState(), optimizedAngularEndState, current.getThetaState())
        );
    }

    public boolean isFinished(double time) {
        return xController.isFinished(time) &&
            yController.isFinished(time) &&
            thetaController.isFinished(time);
    }

    @Override
    public void close() {
        if (xController != null) {
            xController.close();
        }

        if (yController != null) {
            yController.close();
        }
        
        if (thetaController != null) {
            thetaController.close();
        }
    }
}
