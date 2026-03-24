package frc.robot.util;

import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.TurretConstants;

public class RobotState {

    private final NetworkTable loggingTable;
    private final StructPublisher<Pose2d> robotPosePublisher;
    private final StructPublisher<Pose2d> turretOdomPosePublisher;
    private final StructPublisher<Translation2d> robotVelocityOnFieldPublisher;
    private final StructPublisher<Translation3d> robotVelocityAsVisionTargetPublisher;
    private final DoublePublisher hoodFudgeFactorPublisher;
    private final DoublePublisher flywheelFudgeFactorPublisher;
    private final DoublePublisher redHubToRobotCenterPublisher;
    private final DoublePublisher blueHubToRobotCenterPublisher;
    private final DoublePublisher redTagToRobotCenterPublisher;
    private final DoublePublisher blueTagToRobotCenterPublisher;
    private final DoublePublisher redHubToTurretCenterPublisher;
    private final DoublePublisher blueHubToTurretCenterPublisher;
    private final DoublePublisher redTagToTurretCenterPublisher;
    private final DoublePublisher blueTagToTurretCenterPublisher;

    private Supplier<Pose2d> robotOdomPoseSupplier;
    private Supplier<ChassisSpeeds> robotChassisSpeedsSupplier;
    private Supplier<Rotation2d> turretRotationSupplier;

    private Pose2d calculatedTurretOdomPose;
    private boolean currentIntakeState;

    private InterpolatingDoubleTreeMap turretDistanceToHoodAngleMap;
    private InterpolatingDoubleTreeMap turretDistanceToFlywheelVelocityMap;
    private InterpolatingDoubleTreeMap turretDistanceToTimeOfFlightMap;
    private double hoodAngleFudgeFactor;
    private double flywheelSpeedFudgeFactor;

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
        turretDistanceToTimeOfFlightMap.put(6.0, 0.9);
        turretDistanceToTimeOfFlightMap.put(5.0, 0.8);
        turretDistanceToTimeOfFlightMap.put(4.0, 0.7);
        turretDistanceToTimeOfFlightMap.put(3.0, 0.7);
        turretDistanceToTimeOfFlightMap.put(2.0, 0.8);
        turretDistanceToTimeOfFlightMap.put(1.0, 0.8);

        currentIntakeState = IntakeConstants.INTAKE_DEPLOYED_BY_DEFAULT;

        flywheelSpeedFudgeFactor = FlywheelConstants.DEFAULT_FUDGE_FACTOR;
        hoodAngleFudgeFactor = HoodConstants.DEFAULT_FUDGE_FACTOR;

        loggingTable = NetworkTableInstance.getDefault().getTable("RobotState");
        robotPosePublisher = loggingTable.getStructTopic("Robot Odometry Pose", Pose2d.struct).publish();
        turretOdomPosePublisher = loggingTable.getStructTopic("Turret Odometry Pose", Pose2d.struct).publish();
        hoodFudgeFactorPublisher = loggingTable.getDoubleTopic("Current Hood Fudge Factor").publish();
        flywheelFudgeFactorPublisher = loggingTable.getDoubleTopic("Current Flywheel Fudge Factor").publish();
        robotVelocityOnFieldPublisher = loggingTable.getStructTopic("Robot Velocity Vector", Translation2d.struct)
                .publish();
        robotVelocityAsVisionTargetPublisher = loggingTable
                .getStructTopic("Robot Velocity Vector Offset By Robot", Translation3d.struct).publish();

        redHubToRobotCenterPublisher = loggingTable.getDoubleTopic("Distances/Red Hub To Robot Center").publish();
        blueHubToRobotCenterPublisher = loggingTable.getDoubleTopic("Distances/Blue Hub To Robot Center").publish();
        redTagToRobotCenterPublisher = loggingTable.getDoubleTopic("Distances/Red Tag To Robot Center").publish();
        blueTagToRobotCenterPublisher = loggingTable.getDoubleTopic("Distances/Blue Tag To Robot Center").publish();
        redHubToTurretCenterPublisher = loggingTable.getDoubleTopic("Distances/Red Hub To Turret Center").publish();
        blueHubToTurretCenterPublisher = loggingTable.getDoubleTopic("Distances/Blue Hub To Turret Center").publish();
        redTagToTurretCenterPublisher = loggingTable.getDoubleTopic("Distances/Red Tag To Turret Center").publish();
        blueTagToTurretCenterPublisher = loggingTable.getDoubleTopic("Distances/Blue Tag To Turret Center").publish();
    }

    public static synchronized RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }

        return instance;
    }

    public Trigger isBlueAlliance = new Trigger(
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);

    public void setAllSuppliers(Supplier<Pose2d> robotOdomPoseSupplier,
            Supplier<ChassisSpeeds> robotChassisSpeedsSupplier, Supplier<Rotation2d> turretRotationSupplier) {
        this.robotOdomPoseSupplier = robotOdomPoseSupplier;
        this.robotChassisSpeedsSupplier = robotChassisSpeedsSupplier;
        this.turretRotationSupplier = turretRotationSupplier;
    }

    public void logAllInputs() {
        robotPosePublisher.accept(this.getRobotOdomPose());
        turretOdomPosePublisher.accept(this.getTurretOdomPose());
        hoodFudgeFactorPublisher.accept(this.hoodAngleFudgeFactor);
        flywheelFudgeFactorPublisher.accept(this.flywheelSpeedFudgeFactor);

        Translation2d currentRobotVelocity = getFieldRelativeRobotVelocityVector();
        robotVelocityOnFieldPublisher.accept(currentRobotVelocity);
        robotVelocityAsVisionTargetPublisher.accept(
                new Translation3d(
                        currentRobotVelocity.plus(getRobotOdomPose().getTranslation()))
                        .plus(new Translation3d(0.0, 0.0, 0.5)));
    }

    public void logAllDistances() {
        Translation2d robotCenter = this.getRobotOdomPose().getTranslation();
        Translation2d turretCenter = this.getTurretOdomPose().getTranslation();
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        redHubToRobotCenterPublisher
                .accept(robotCenter.getDistance(FieldConstants.RED_ALLIANCE_HUB_POSE.getTranslation()));
        blueHubToRobotCenterPublisher
                .accept(robotCenter.getDistance(FieldConstants.BLUE_ALLIANCE_HUB_POSE.getTranslation()));
        redTagToRobotCenterPublisher
                .accept(robotCenter.getDistance(fieldLayout.getTagPose(10).get().toPose2d().getTranslation()));
        blueTagToRobotCenterPublisher
                .accept(robotCenter.getDistance(fieldLayout.getTagPose(26).get().toPose2d().getTranslation()));

        redHubToTurretCenterPublisher
                .accept(turretCenter.getDistance(FieldConstants.RED_ALLIANCE_HUB_POSE.getTranslation()));
        blueHubToTurretCenterPublisher
                .accept(turretCenter.getDistance(FieldConstants.BLUE_ALLIANCE_HUB_POSE.getTranslation()));
        redTagToTurretCenterPublisher
                .accept(turretCenter.getDistance(fieldLayout.getTagPose(10).get().toPose2d().getTranslation()));
        blueTagToTurretCenterPublisher
                .accept(turretCenter.getDistance(fieldLayout.getTagPose(26).get().toPose2d().getTranslation()));
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
                poseToSetAngleFrom.getTranslation().getDistance(this.getTurretOdomPose().getTranslation()))
                + hoodAngleFudgeFactor;
    }

    public Command incrementHoodFudgeFactor = Commands.runOnce(
            () -> this.hoodAngleFudgeFactor += HoodConstants.FUDGE_INCREMENT_VALUE);
    public Command decrementHoodFudgeFactor = Commands.runOnce(
            () -> this.hoodAngleFudgeFactor -= HoodConstants.FUDGE_INCREMENT_VALUE);
    public Command resetHoodFudgeFactor = Commands.runOnce(
            () -> this.hoodAngleFudgeFactor = HoodConstants.DEFAULT_FUDGE_FACTOR);

    public double getFlywheelVelocityFromGoalPose(Pose2d poseToSetAngleFrom) {
        // Get distance between turret position and goal position, plug that into the
        // map
        return turretDistanceToFlywheelVelocityMap.get(
                poseToSetAngleFrom.getTranslation().getDistance(this.getTurretOdomPose().getTranslation()))
                + flywheelSpeedFudgeFactor;
    }

    public Command incrementFlywheelFudgeFactor = Commands.runOnce(
            () -> this.flywheelSpeedFudgeFactor += FlywheelConstants.FUDGE_INCREMENT_VALUE);
    public Command decrementFlywheelFudgeFactor = Commands.runOnce(
            () -> this.flywheelSpeedFudgeFactor -= FlywheelConstants.FUDGE_INCREMENT_VALUE);
    public Command resetFlywheelSpeedFudgeFactor = Commands.runOnce(
            () -> this.flywheelSpeedFudgeFactor = FlywheelConstants.DEFAULT_FUDGE_FACTOR);

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

    public Translation2d getFieldRelativeRobotVelocityVector() {
        ChassisSpeeds currentSpeeds = robotChassisSpeedsSupplier.get();
        return new Translation2d(
                currentSpeeds.vxMetersPerSecond,
                currentSpeeds.vyMetersPerSecond)
                .rotateBy(getRobotOdomPose().getRotation());
    }

    public ShootOnTheMoveOutputPoses calculateParametersForShootOnTheMove(Translation2d goalTranslationOnField) {

        ShootOnTheMoveOutputPoses outputData = new ShootOnTheMoveOutputPoses();

        Translation2d turretTranslationOnField = this.getTurretOdomPose().getTranslation();
        Translation2d turretVelocityOnFieldAsTranslation = this.getFieldRelativeRobotVelocityVector();

        double displacement_x, displacement_y, distance_derived_from_current_time;

        double turret_velocity_x, turret_velocity_y;
        turret_velocity_x = turretVelocityOnFieldAsTranslation.getX();
        turret_velocity_y = turretVelocityOnFieldAsTranslation.getY();

        // Use Speed = Distance / Time
        double initial_distance_to_target = goalTranslationOnField.getDistance(turretTranslationOnField);
        double horizontal_projectile_velocity = initial_distance_to_target
                / turretDistanceToTimeOfFlightMap.get(initial_distance_to_target);
                
        Translation2d projectileVelocityOnFieldAsTranslation;
        projectileVelocityOnFieldAsTranslation = new Translation2d(
                horizontal_projectile_velocity,
                PhotonUtils.getYawToPose(new Pose2d(turretTranslationOnField, new Rotation2d()),
                        new Pose2d(goalTranslationOnField, new Rotation2d())));

        double current_time_guess, next_time_guess;

        double error, error_derivative;

        // Time = Distance / Speed
        current_time_guess = goalTranslationOnField.getDistance(turretTranslationOnField) /
                projectileVelocityOnFieldAsTranslation.plus(turretVelocityOnFieldAsTranslation)
                        .getDistance(new Translation2d(0, 0));

        for (int iteration = 0; iteration < ShootingConstants.MAX_ITERATION_COUNT; iteration++) {
            displacement_x = goalTranslationOnField.getX() - turretTranslationOnField.getX()
                    + (turretVelocityOnFieldAsTranslation.getX() * current_time_guess);
            displacement_y = goalTranslationOnField.getY() - turretTranslationOnField.getY()
                    + (turretVelocityOnFieldAsTranslation.getY() * current_time_guess);
            distance_derived_from_current_time = Math
                    .sqrt((displacement_x * displacement_x) + (displacement_y * displacement_y));

            error = current_time_guess - turretDistanceToTimeOfFlightMap.get(distance_derived_from_current_time);
            error_derivative = 1 + (((displacement_x * turret_velocity_x) + (displacement_y * turret_velocity_y))
                    / (distance_derived_from_current_time * horizontal_projectile_velocity));

            next_time_guess = current_time_guess - (error / error_derivative);

            outputData.setIteratedTranslationWithIndex(iteration, goalTranslationOnField
                    .minus(turretVelocityOnFieldAsTranslation.times(current_time_guess)));

            current_time_guess = next_time_guess;
        }

        outputData.setGoalTranslation(goalTranslationOnField
                .minus(turretVelocityOnFieldAsTranslation.times(current_time_guess)));

        return outputData;
    }
}
