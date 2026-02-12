package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotState {
    
    private static RobotState instance = null;

    private RobotState() {}

    public static synchronized RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }

        return instance;
    }

    private Supplier<Pose2d> robotOdomPoseSupplier;
    private Supplier<Pose2d> turretCameraPoseSupplier;
    private DoubleSupplier turretCameraEstimationTimestampSupplier;
    
    // For Aditya:
    private static final Pose2d ROBOT_TO_TURRET_OFFSET = new Pose2d();
    
    private Pose2d calculatedTurretOdomPose;

    public void setAllPublishers(Supplier<Pose2d> robotOdomPoseSupplier, Supplier<Pose2d> turretCameraPoseSupplier, DoubleSupplier turretCameraEstimationTimestampSupplier) {
        this.robotOdomPoseSupplier = robotOdomPoseSupplier;
        this.turretCameraPoseSupplier = turretCameraPoseSupplier;
        this.turretCameraEstimationTimestampSupplier = turretCameraEstimationTimestampSupplier;
    }

    public Pose2d getRobotOdomPose() {
        return robotOdomPoseSupplier.get();
    }

    public Pose2d getTurretOdomPose() {
        return calculatedTurretOdomPose;
    }

    public Pose2d getTurretCameraPose() {
        return turretCameraPoseSupplier.get();
    }

    public double getTurretCameraEstimationTimestamp() {
        return turretCameraEstimationTimestampSupplier.getAsDouble();
    }

    public void updateTurretPose(Pose2d robotPose, Rotation2d turretRotation) {
        // Step Uno: Create initial turret position by adding offset to robot position
        Pose2d turretOnField = new Pose2d(
            robotPose.getTranslation()
                .plus(ROBOT_TO_TURRET_OFFSET.getTranslation()),
            new Rotation2d()
        );
        
        // Step Dos: Rotate turret position around robot center to account for robot rotation
        Pose2d estimatedTurretPose = turretOnField.rotateAround(
            robotPose.getTranslation(),
            robotPose.getRotation()
        );
        
        // Step 3: Rotate around turret's own position to account for turret's rotation
        estimatedTurretPose = estimatedTurretPose.rotateAround(
            estimatedTurretPose.getTranslation(), 
            turretRotation
        );
        
        // Step the Last: Update the stored turret pose
        this.calculatedTurretOdomPose = estimatedTurretPose;
    }
}
