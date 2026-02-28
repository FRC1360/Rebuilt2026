package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.TurretConstants;

public class RobotState {

    private Supplier<Pose2d> robotOdomPoseSupplier;
    private Supplier<Rotation2d> turretRotationSupplier;
    private Supplier<Pose2d> turretCameraPoseSupplier;
    private DoubleSupplier turretCameraEstimationTimestampSupplier;

    private Pose2d calculatedTurretOdomPose;

    private InterpolatingDoubleTreeMap turretDistanceToHoodAngleMap;
    private InterpolatingDoubleTreeMap turretDistanceToTimeOfFlightMap;

    private static RobotState instance = null;

    private RobotState() {
        turretDistanceToHoodAngleMap = new InterpolatingDoubleTreeMap();
        turretDistanceToHoodAngleMap.put(0.0, 0.0);

        turretDistanceToTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        turretDistanceToTimeOfFlightMap.put(0.0, 0.0);
    }

    public static synchronized RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }

        return instance;
    }

    public void setAllSuppliers(Supplier<Pose2d> robotOdomPoseSupplier, Supplier<Rotation2d> turretRotationSupplier,
            Supplier<Pose2d> turretCameraPoseSupplier, DoubleSupplier turretCameraEstimationTimestampSupplier) {
        this.robotOdomPoseSupplier = robotOdomPoseSupplier;
        this.turretRotationSupplier = turretRotationSupplier;
        this.turretCameraPoseSupplier = turretCameraPoseSupplier;
        this.turretCameraEstimationTimestampSupplier = turretCameraEstimationTimestampSupplier;
    }

    public double getHoodAngleFromGoalPose(Pose2d poseToSetAngleFrom) {
        // Get distance between turret position and goal position, plug that into the map
        return turretDistanceToHoodAngleMap.get(
                poseToSetAngleFrom.getTranslation().getDistance(this.getTurretOdomPose().getTranslation()));
    }

    public Pose2d getRobotOdomPose() {
        return robotOdomPoseSupplier.get();
    }

    public Rotation2d getTurretRotation() {
        return turretRotationSupplier.get();
    }

    public Pose2d getTurretOdomPose() {
        updateTurretPose(
                this.getRobotOdomPose(),
                this.getTurretRotation());
        return calculatedTurretOdomPose;
    }

    public Pose2d getTurretCameraPose() {
        return turretCameraPoseSupplier.get();
    }

    public double getTurretCameraEstimationTimestamp() {
        return turretCameraEstimationTimestampSupplier.getAsDouble();
    }

    private void updateTurretPose(Pose2d robotPose, Rotation2d turretRotation) {
        /*
         * Step Uno: Create initial turret position by adding offset to robot position
         * Note: Transform affects it in the following way:
         * - X value becomes robot relative forward/backward
         * - Y value becomes robot relative left/right
         * - Z value rotates the shifted pose around it's own center
         */
        Pose2d estimatedTurretPose = robotPose.transformBy(TurretConstants.ROBOT_TO_TURRET_CENTER);

        /*
         * Step Dos: Rotate around turret's own position to account for turret's
         * rotation
         */
        estimatedTurretPose = estimatedTurretPose.rotateAround(
                estimatedTurretPose.getTranslation(),
                turretRotation);

        // Step the Third: Update the stored turret pose
        this.calculatedTurretOdomPose = estimatedTurretPose;
    }
}
