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
    private Supplier<Pose2d> turretOdomPoseSupplier;
    private Supplier<Pose2d> turretCameraPoseSupplier;
    private DoubleSupplier turretCameraEstimationTimestampSupplier;
    private Supplier<Pose2d> robotToTurretOffsetSupplier;

    private Pose2d calculatedTurretOdomPose;

    public Pose2d getRobotOdomPose() {
        return robotOdomPoseSupplier.get();
    }

    public void setRobotOdomPoseSupplier(Supplier<Pose2d> robotOdomPoseSupplier) {
        this.robotOdomPoseSupplier = robotOdomPoseSupplier;
    }

    public Pose2d getTurretOdomPose() {
        return turretOdomPoseSupplier.get();
    }

    public void setTurretOdomPoseSupplier(Supplier<Pose2d> turretOdomPoseSupplier) {
        this.turretOdomPoseSupplier = turretOdomPoseSupplier;
    }

    public Pose2d getTurretCameraPose() {
        return turretCameraPoseSupplier.get();
    }

    public void setTurretCameraPoseSupplier(Supplier<Pose2d> turretCameraPoseSupplier) {
        this.turretCameraPoseSupplier = turretCameraPoseSupplier;
    }

    public double getTurretCameraEstimationTimestamp() {
        return turretCameraEstimationTimestampSupplier.getAsDouble();
    }

    public void setTurretCameraEstimationTimestampSupplier(DoubleSupplier turretCameraEstimationTimestampSupplier) {
        this.turretCameraEstimationTimestampSupplier = turretCameraEstimationTimestampSupplier;
    }

    public void setRobotToTurretOffsetSupplier(Supplier<Pose2d> robotToTurretOffsetSupplier) {
        this.robotToTurretOffsetSupplier = robotToTurretOffsetSupplier;
    }


    // Not sure if this is needed, but otherwise the variable never gets used
    public Pose2d getCalculatedTurretOdomPose() {
        return calculatedTurretOdomPose;
    }


    public void updateTurretPose(Pose2d robotPose, Rotation2d turretRotation) {
        Pose2d robotToTurretOffset = robotToTurretOffsetSupplier.get();
        // Step Uno: Create initial turret position by adding offset to robot position
        Pose2d turretOnField = new Pose2d(
            robotPose.getTranslation()
                .plus(robotToTurretOffset.getTranslation()),
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
