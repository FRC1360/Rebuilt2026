package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;

public class RobotState {

    private final NetworkTable loggingTable;
    private final StructPublisher<Pose2d> robotPosePublisher;
    private final StructPublisher<Pose2d> turretOdomPosePublisher;

    private Supplier<Pose2d> robotOdomPoseSupplier;
    private Supplier<Rotation2d> turretRotationSupplier;

    private Pose2d calculatedTurretOdomPose;
    private boolean currentIntakeState;

    private InterpolatingDoubleTreeMap turretDistanceToHoodAngleMap;
    private InterpolatingDoubleTreeMap turretDistanceToFlywheelVelocityMap;
    private InterpolatingDoubleTreeMap turretDistanceToTimeOfFlightMap;

    private static RobotState instance = null;

    private RobotState() {
        turretDistanceToHoodAngleMap = new InterpolatingDoubleTreeMap();
        // turretDistanceToHoodAngleMap.put(1.423, 74.0);
        // turretDistanceToHoodAngleMap.put(2.511, 71.0);
        // turretDistanceToHoodAngleMap.put(2.75, 70.0);
        // turretDistanceToHoodAngleMap.put(3.54, 65.0);
        // turretDistanceToHoodAngleMap.put(4.26, 63.0);
        // turretDistanceToHoodAngleMap.put(5.016, 60.0);
        turretDistanceToHoodAngleMap.put(1.445, 68.0);
        turretDistanceToHoodAngleMap.put(2.585, 62.0);
        turretDistanceToHoodAngleMap.put(3.017, 61.0);
        turretDistanceToHoodAngleMap.put(3.489, 60.0);
        turretDistanceToHoodAngleMap.put(4.085, 59.0);
        turretDistanceToHoodAngleMap.put(5.033, 57.0);

        turretDistanceToFlywheelVelocityMap = new InterpolatingDoubleTreeMap();
        // turretDistanceToFlywheelVelocityMap.put(1.423, 50.0);
        // turretDistanceToFlywheelVelocityMap.put(2.511,55.0);
        // turretDistanceToFlywheelVelocityMap.put(2.75, 56.0);
        // turretDistanceToFlywheelVelocityMap.put(3.54, 58.0);
        // turretDistanceToFlywheelVelocityMap.put(4.26, 60.0);
        // turretDistanceToFlywheelVelocityMap.put(5.016, 65.0);
        turretDistanceToFlywheelVelocityMap.put(1.445, 47.0);
        turretDistanceToFlywheelVelocityMap.put(2.585, 50.0);
        turretDistanceToFlywheelVelocityMap.put(3.017, 52.0);
        turretDistanceToFlywheelVelocityMap.put(3.489, 55.0);
        turretDistanceToFlywheelVelocityMap.put(4.085, 60.0);
        turretDistanceToFlywheelVelocityMap.put(5.033, 65.0);

        turretDistanceToTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        turretDistanceToTimeOfFlightMap.put(0.0, 0.0);

        currentIntakeState = IntakeConstants.INTAKE_DEPLOYED_BY_DEFAULT;

        loggingTable = NetworkTableInstance.getDefault().getTable("RobotState");
        robotPosePublisher = loggingTable.getStructTopic("Robot Odometry Pose", Pose2d.struct).publish();
        turretOdomPosePublisher = loggingTable.getStructTopic("Turret Odometry Pose", Pose2d.struct).publish();
    }

    public static synchronized RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }

        return instance;
    }

    public Trigger isBlueAlliance = new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);

    public void setAllSuppliers(Supplier<Pose2d> robotOdomPoseSupplier, Supplier<Rotation2d> turretRotationSupplier) {
        this.robotOdomPoseSupplier = robotOdomPoseSupplier;
        this.turretRotationSupplier = turretRotationSupplier;
    }

    public void logAllInputs() {
        robotPosePublisher.accept(this.getRobotOdomPose());
        turretOdomPosePublisher.accept(this.getTurretOdomPose());
    }

    public final Trigger isIntakeCurrentlyDeployed = new Trigger(() -> this.currentIntakeState);

    public Command toggleIntakeState = Commands.runOnce(() -> {
        this.currentIntakeState = !this.currentIntakeState;
    });

    public Command setIntakeState(boolean intakeActivated) {
        return Commands.runOnce(() -> {
            this.currentIntakeState = intakeActivated;
        });
    }

    public double getHoodAngleFromGoalPose(Pose2d poseToSetAngleFrom) {
        // Get distance between turret position and goal position, plug that into the
        // map
        return turretDistanceToHoodAngleMap.get(
                poseToSetAngleFrom.getTranslation().getDistance(this.getTurretOdomPose().getTranslation()));
    }

    public double getFlywheelVelocityFromGoalPose(Pose2d poseToSetAngleFrom) {
        // Get distance between turret position and goal position, plug that into the
        // map
        return turretDistanceToFlywheelVelocityMap.get(
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

    private void updateTurretPose(Pose2d robotPose, Rotation2d turretRotation) {
        /*
         * Step Uno: Create initial turret position by adding offset to robot position
         * Note: Transform affects it in the following way:
         * - X value becomes robot relative forward/backward
         * - Y value becomes robot relative left/right
         * - Z value rotates the shifted pose around it's own center
         */
        Pose2d estimatedTurretPose = robotPose.transformBy(
                new Transform2d(
                        TurretConstants.ROBOT_TO_TURRET_CENTER.getTranslation(),
                        new Rotation2d()));

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
