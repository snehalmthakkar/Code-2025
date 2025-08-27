package com.team6962.lib.swerve.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class LineFollowing {
    private LineFollowing() {
    }

    public static class Vector2 {
        public final double x;
        public final double y;

        public Vector2(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Vector2() {
            this(0, 0);
        }

        public Vector2(Translation2d translation) {
            this(translation.getX(), translation.getY());
        }

        public Vector2(Pose2d pose) {
            this(pose.getTranslation());
        }

        public Vector2(ChassisSpeeds speeds) {
            this(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        }

        public double magnitude() {
            return Math.hypot(x, y);
        }

        public Vector2 plus(Vector2 other) {
            return new Vector2(this.x + other.x, this.y + other.y);
        }

        public Vector2 minus(Vector2 other) {
            return new Vector2(this.x - other.x, this.y - other.y);
        }

        public Vector2 times(double scalar) {
            return new Vector2(this.x * scalar, this.y * scalar);
        }

        public double dot(Vector2 other) {
            return this.x * other.x + this.y * other.y;
        }

        public Vector2 rotate90CCW() {
            return new Vector2(-this.y, this.x);
        }

        public Vector2 rotate90CW() {
            return new Vector2(this.y, -this.x);
        }

        public Vector2 unit() {
            double mag = magnitude();

            if (mag < 1e-9) {
                return new Vector2(0, 0);
            }

            return new Vector2(this.x / mag, this.y / mag);
        }

        public Translation2d toTranslation2d() {
            return new Translation2d(this.x, this.y);
        }

        public Pose2d toPose2d(Rotation2d rotation) {
            return new Pose2d(this.x, this.y, rotation);
        }
    }

    public static Vector2 directionBetweenPoints(Vector2 start, Vector2 end) {
        return end.minus(start).unit();
    }

    public static Vector2 fieldToPathPosition(Vector2 robotPositionFieldCoords, Vector2 start, Vector2 pathDirection) {
        return fieldToPathDisplacement(robotPositionFieldCoords.minus(start), pathDirection);
    }

    public static Vector2 pathToFieldPosition(Vector2 robotPositionPathCoords, Vector2 start, Vector2 pathDirection) {
        return start.plus(pathToFieldDisplacement(start, pathDirection));
    }

    public static Vector2 fieldToPathDisplacement(Vector2 robotDisplacementFieldCoords, Vector2 pathDirection) {
        Vector2 pathNormal = pathDirection.rotate90CCW();

        return new Vector2(
            robotDisplacementFieldCoords.dot(pathDirection),
            robotDisplacementFieldCoords.dot(pathNormal)
        );
    }

    public static Vector2 pathToFieldDisplacement(Vector2 robotDisplacementPathCoords, Vector2 pathDirection) {
        Vector2 pathNormal = pathDirection.rotate90CCW();

        return pathDirection.times(robotDisplacementPathCoords.x).plus(
            pathNormal.times(robotDisplacementPathCoords.y)
        );
    }
}
