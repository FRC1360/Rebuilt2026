// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
        public static final int TURRET_MOTOR_ID = 10;
        public static final double ENCODER_STARTUP_ANGLE_DEGREES = 0.0;
        public static final double VELOCITY_CONVERSION_FACTOR = 0;
        public static final double POSITION_CONVERSION_FACTOR = 0;
        public static final double TOTAL_WRAP_AROUND_ANGLE_RANGE = 400;
        public static final double POSITIVE_THRESHOLD = 180 - (TOTAL_WRAP_AROUND_ANGLE_RANGE - 360) / 2;
        public static final double NEGATIVE_THRESHOLD = -180 + (TOTAL_WRAP_AROUND_ANGLE_RANGE - 360) / 2;

        public static final double GEAR_RATIO = (1.0 / 7.0) * 360.0;

        public static final double MAX_V = 3000.0;
        public static final double MAX_A = 3500.0;
        public static final double KP = 0.055;
        public static final double KI = 0.01;
        public static final double KD = 0.0;
        public static final double KS = 0.11;
        public static final double KV = 0.0022;
        public static final double KA = 0.0;
        public static final double KG = 0.0;

        public static final Pose2d ROBOT_TO_TURRET = 
            new Pose2d(
                new Translation2d(0.225, 0.0),
                new Rotation2d()
            );
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