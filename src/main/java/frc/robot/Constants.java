// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class HoodConstants {
        public static final int KRAKEN_CAN_ID = 41;
        public static final int KRAKEN_STATOR_CURRENT_LIMIT = 80;
        public static final int KRAKEN_SUPPLY_CURRENT_LIMIT = 30;
        public static final InvertedValue KRAKEN_INVERTED_VALUE = InvertedValue.Clockwise_Positive;

        public static final double STARTUP_ANGLE_DEGREES = 75.0;
        public static final double MECHANISM_CONVERSION_FACTOR = ((164.0 / 12.0) * (60.0 / 18.0)) / 360.0;
        public static final double PID_TOLERANCE = 2.0;

        public static final double MIN_ANGLE = 57.0;
        public static final double MAX_ANGLE = 74.0;

        public static final double DEFAULT_FUDGE_FACTOR = -1.0;
        public static final double FUDGE_INCREMENT_VALUE = 0.5;
    }

    public static final class IntakeConstants {
        // Roller Constants
        public static final int ROLLER_VORTEX_CAN_ID = 31;
       // public static final int ROLLER_VORTEX_STALL_CURRENT_LIMIT = 80;
        public static final int ROLLER_VORTEX_STATOR_CURRENT_LIMIT = 80;
        //CHANGED THIS BTW
        public static final InvertedValue ROLLER_VORTEX_INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final double ROLLER_ACTIVATED_SPEED = 0.7;

        // Pivot Constants
        public static final int PIVOT_VORTEX_CAN_ID = 30;
        public static final int PIVOT_VORTEX_STALL_CURRENT_LIMIT = 40;
        public static final int PIVOT_VORTEX_FREE_CURRENT_LIMIT = 40;
        
        public static final boolean PIVOT_VORTEX_INVERTED = true;

        // Position units: degrees, Velocity units: degrees/sec
        public static final double PIVOT_POSITION_CONVERSION_RATIO = (1.0 / 64.28571) * 360.0;
        public static final double PIVOT_VELOCITY_CONVERSION_RATIO = ((1.0 / 64.28571) * 360.0) / 60.0;

        public static final double PIVOT_STARTUP_ANGLE = 121.3;
        public static final double PIVOT_PID_TOLERANCE = 2.0;
        public static final double PIVOT_RETRACTED_ANGLE = 120.0;
        public static final double PIVOT_DEPLOYED_ANGLE = 10.0;
        public static final double PIVOT_DEPLOYED_AND_ACTIVATED_ANGLE = -0.4;

        // Used in RobotState to manage toggling functionality
        public static final boolean INTAKE_DEPLOYED_BY_DEFAULT = false;
    }

    public static final class TurretConstants {
        public static final int KRAKEN_CAN_ID = 40;
        public static final InvertedValue KRAKEN_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
        public static final int KRAKEN_STATOR_CURRENT_LIMIT = 80;
        public static final double MECHANISM_CONVERSION_FACTOR = (27.125) / 360.0;

        public static final double ENCODER_STARTUP_ANGLE_DEGREES = 165;
        public static final double TOTAL_WRAP_AROUND_ANGLE_RANGE = 390.0;
        public static final double POSITIVE_THRESHOLD = 180 - (TOTAL_WRAP_AROUND_ANGLE_RANGE - 360) / 2;
        public static final double NEGATIVE_THRESHOLD = -180 + (TOTAL_WRAP_AROUND_ANGLE_RANGE - 360) / 2;
        public static final double PID_TOLERANCE = 3;

        // in +- degrees, ex: 15 degrees would accept 15 deg higher and lower
        public static final double PASSING_PID_TOLERANCE = 7.5;

        public static final Transform2d ROBOT_TO_TURRET_CENTER = new Transform2d(
                new Translation2d(-((0.705 / 2.0) - 0.268852), ((0.705 / 2.0) - 0.212725)),
                Rotation2d.fromDegrees(285.0));
        /*
         * Note: The rotation offset for turret to center is measured relative to where
         * the encoder
         * reading will be zero.
         * Zero degrees on the encoder is ALWAYS OPPOSITE the MIDDLE of the wraparound
         * range.
         * The startup angle degrees is the delta between wraparound opposite and where
         * the turret
         * actually starts.
         */
    }

    public static final class IndexConstants {
        public static final int HOPPER_VORTEX_CAN_ID = 32;
        public static final int HOPPER_VORTEX_STALL_CURRENT_LIMIT = 80;
        public static final int HOPPER_VORTEX_FREE_CURRENT_LIMIT = 80;
        public static final boolean HOPPER_VORTEX_INVERTED = true;

        public static final int MAGAZINE_VORTEX_CAN_ID = 33;
        public static final int MAGAZINE_VORTEX_STALL_CURRENT_LIMIT = 45;
        public static final int MAGAZINE_VORTEX_FREE_CURRENT_LIMIT = 45;
        public static final boolean MAGAZINE_VORTEX_INVERTED = false;

        public static final int MAGAZINE_SENSOR_DIGITAL_CHANNEL = 1;

        public static final double HOPPER_SPEED = 0.9;
        public static final double MAGAZINE_SPEED = 0.9;
    }

    public static final class FlywheelConstants {
        public static final int VORTEX_LEADER_CAN_ID = 42;
        public static final int VORTEX_FOLLOWER_CAN_ID = 43;

        public static final double PID_TOLERANCE = 5.0;
        public static final double HIGH_SPEED = 0;
        public static final double LOW_SPEED = 0;
        public static final double DISTANCE_THRESHOLD = 0;
        public static final int FLYWHEEL_STALL_CURRENT_LIMIT = 40;
        public static final int FLYWHEEL_FREE_CURRENT_LIMIT = 40;
        public static final boolean FLYWHEEL_LEADER_INVERTED = false;
        public static final boolean FLYWHEEL_FOLLOWER_INVERTED = false;
        public static final double VELOCITY_CONVERSION_FACTOR = (4 * Math.PI) / (60 * 12);
        public static final double POSITION_CONVERSION_FACTOR = (4 * Math.PI) / (12);

        public static final double DEFAULT_FUDGE_FACTOR = 1.0;
        public static final double FUDGE_INCREMENT_VALUE = 0.5;
    }

    public static final class DrivetrainConstants {
        public static final Transform3d BACK_LEFT_SWERVECAM_ROBOT_RELATIVE_TRANSFORM = new Transform3d(
                new Translation3d(-((0.704850 / 2.0) - 0.094180), (0.704850 / 2.0) - 0.094180, 0.208540),
                new Rotation3d(0, -Math.toRadians(10), Math.toRadians(134.136029)));
        public static final Transform3d BACK_RIGHT_BACK_SWERVECAM_ROBOT_RELATIVE_TRANSFORM = new Transform3d(
                new Translation3d(-0.288, -0.288, 0.241),
                new Rotation3d(0, -Math.toRadians(27), Math.toRadians(180.0)));
        public static final Transform3d BACK_RIGHT_RIGHT_SWERVECAM_ROBOT_RELATIVE_TRANSFORM = new Transform3d(
                new Translation3d(-0.279, -0.309, 0.241),
                new Rotation3d(0, -Math.toRadians(12), Math.toRadians(-90.0)));
    }

    public static final class ShootingConstants {
        /*
         * Units: m/s
         * Function: Used in the following:
         * - Initial time guess for the newton recursion
         * - Approximation in the derivative function for change in distance
         * with respect to change in time -> which gets used in the error function
         */
        public static final double HORIZONTAL_BALL_SPEED = 3.0;

        /*
         * Maximum amount of iterations for the newton recursion
         */
        public static final int MAX_ITERATION_COUNT = 5;
    }
}