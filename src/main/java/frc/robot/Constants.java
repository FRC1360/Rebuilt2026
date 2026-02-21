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

    public static final class IntakeConstants {
        // Roller Constants
        public static final int ROLLER_VORTEX_CAN_ID = 60;
        public static final int ROLLER_VORTEX_STALL_CURRENT_LIMIT = 30;
        public static final int ROLLER_VORTEX_FREE_CURRENT_LIMIT = 30;
        public static final boolean ROLLER_VORTEX_INVERTED = false;

        public static final double ROLLER_ACTIVATED_SPEED = 0.5;

        // Pivot Constants
        public static final int PIVOT_VORTEX_CAN_ID = 0;
        public static final int PIVOT_VORTEX_STALL_CURRENT_LIMIT = 30;
        public static final int PIVOT_VORTEX_FREE_CURRENT_LIMIT = 30;
        public static final boolean PIVOT_VORTEX_INVERTED = false;

        public static final double PIVOT_POSITION_CONVERSION_RATIO = 0.0;
        public static final double PIVOT_VELOCITY_CONVERSION_RATIO = 0.0;
        public static final double PIVOT_STARTUP_ANGLE = 0;
        public static final double PIVOT_RETRACTED_ANGLE = 0.0;
        public static final double PIVOT_DEPLOYED_ANGLE = 2.0;
        public static final double PIVOT_PID_TOLERANCE = 2.0;
    }

    public static class TurretConstants {
        public static final int KRAKEN_CAN_ID = 10;
        public static final InvertedValue KRAKEN_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
        public static final double MECHANISM_CONVERSION_FACTOR = (1.0 / 7.0) * 360.0;

        public static final double ENCODER_STARTUP_ANGLE_DEGREES = 0.0;
        public static final double TOTAL_WRAP_AROUND_ANGLE_RANGE = 400;
        public static final double POSITIVE_THRESHOLD = 180 - (TOTAL_WRAP_AROUND_ANGLE_RANGE - 360) / 2;
        public static final double NEGATIVE_THRESHOLD = -180 + (TOTAL_WRAP_AROUND_ANGLE_RANGE - 360) / 2;

        public static final Transform2d ROBOT_TO_TURRET_CENTER = new Transform2d(
                new Translation2d(0.225, 0.0),
                new Rotation2d());
        public static final Transform3d TURRET_CENTER_TO_CAMERA = new Transform3d(
                new Translation3d(0.05, 0, 0.045),
                new Rotation3d(0, -Math.toRadians(10), Math.toRadians(0)));

        /*
         * Note: The rotation offset for turret to center is measured relative to where the encoder
         * reading will be zero. 
         * Zero degrees on the encoder is ALWAYS OPPOSITE the MIDDLE of the wraparound range.
         * The startup angle degrees is the delta between wraparound opposite and where the turret
         * actually starts.
         */
    }

    public static class IndexConstants {
        public static final int HOPPER_CONVEYOR_ID = 0;
        public static final int MAGAZINE_CONVEYOR_ID = 0;
        public static final int MAGAZINE_SENSOR_ID = 1;

        public static final double HOPPER_SPEED = 0.5;
        public static final double MAGAZINE_TARGET_SPEED = 0.0;
    }

    public static class FlywheelConstants {
        public static final double HIGH_SPEED = 0;
        public static final double LOW_SPEED = 0;
        public static final double DISTANCE_THRESHOLD = 0;
    }
}