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
    private final StructPublisher<Translation2d> turretRotationalVelocityOnFieldPublisher;
    private final StructPublisher<Translation3d> turretRotationalVelocityAsVisionTargetPublisher;
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

    private final AprilTagFieldLayout fieldLayout;

    private Supplier<Pose2d> robotOdomPoseSupplier;
    private Supplier<ChassisSpeeds> robotChassisSpeedsSupplier;
    private Supplier<Rotation2d> turretRotationSupplier;

    private Pose2d calculatedTurretOdomPose;
    private boolean currentIntakeState;

    private InterpolatingDoubleTreeMap turretDistanceToHoodAngleMap;
    private InterpolatingDoubleTreeMap turretDistanceToFlywheelVelocityMap;
    private InterpolatingDoubleTreeMap turretDistanceToTimeOfFlightMap;
    private InterpolatingDoubleTreeMap turretDistanceToHoodAnglePassingMap;
    private InterpolatingDoubleTreeMap turretDistanceToFlywheelVelocityPassingMap;
    private InterpolatingDoubleTreeMap turretDistanceToTimeOfFlightPassingMap;
    private double hoodAngleFudgeFactor;
    private double flywheelSpeedFudgeFactor;

    private static RobotState instance = null;

    private RobotState() {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        turretDistanceToHoodAngleMap = new InterpolatingDoubleTreeMap();
        turretDistanceToHoodAngleMap.put(1.53, 73.5);
        turretDistanceToHoodAngleMap.put(2.063, 70.0);
        turretDistanceToHoodAngleMap.put(2.501, 68.0);
        turretDistanceToHoodAngleMap.put(3.049, 66.0);
        turretDistanceToHoodAngleMap.put(3.551, 64.0);
        turretDistanceToHoodAngleMap.put(4.012, 61.5);
        turretDistanceToHoodAngleMap.put(4.47, 59.5);
        turretDistanceToHoodAngleMap.put(5.052, 58.5);
        turretDistanceToHoodAngleMap.put(5.50, 57.0);
        turretDistanceToHoodAngleMap.put(7.0, 49.983);

        turretDistanceToFlywheelVelocityMap = new InterpolatingDoubleTreeMap();
        turretDistanceToFlywheelVelocityMap.put(1.53, 43.0);
        turretDistanceToFlywheelVelocityMap.put(2.063, 45.0);
        turretDistanceToFlywheelVelocityMap.put(2.501, 47.0);
        turretDistanceToFlywheelVelocityMap.put(3.049, 51.0);
        turretDistanceToFlywheelVelocityMap.put(3.551, 53.0);
        turretDistanceToFlywheelVelocityMap.put(4.012, 56.0);
        turretDistanceToFlywheelVelocityMap.put(4.47, 58.0);
        turretDistanceToFlywheelVelocityMap.put(5.052, 63.0);
        turretDistanceToFlywheelVelocityMap.put(5.50, 65.0);
        turretDistanceToFlywheelVelocityMap.put(7.0, 73.567);
        turretDistanceToFlywheelVelocityMap.put(10.0, 91.0);

        turretDistanceToTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        turretDistanceToTimeOfFlightMap.put(1.53, 0.955);
        turretDistanceToTimeOfFlightMap.put(2.063, 0.945);
        turretDistanceToTimeOfFlightMap.put(2.501, 0.9067);
        turretDistanceToTimeOfFlightMap.put(3.049, 0.967);
        turretDistanceToTimeOfFlightMap.put(3.551, 1.083);
        turretDistanceToTimeOfFlightMap.put(4.012, 1.110);
        turretDistanceToTimeOfFlightMap.put(4.47, 1.113);
        turretDistanceToTimeOfFlightMap.put(5.052, 1.240);
        turretDistanceToTimeOfFlightMap.put(5.50, 1.280);
        turretDistanceToTimeOfFlightMap.put(7.0, 1.385);
        turretDistanceToTimeOfFlightMap.put(10.0, 1.66);

        turretDistanceToHoodAnglePassingMap = new InterpolatingDoubleTreeMap();
        turretDistanceToHoodAnglePassingMap.put(4.6, 57.0);
        turretDistanceToHoodAnglePassingMap.put(8.5, 57.0);
        turretDistanceToHoodAnglePassingMap.put(12.0, 57.0);

        turretDistanceToFlywheelVelocityPassingMap = new InterpolatingDoubleTreeMap();
        turretDistanceToFlywheelVelocityPassingMap.put(4.6, 50.0);
        turretDistanceToFlywheelVelocityPassingMap.put(8.5, 75.0);
        turretDistanceToFlywheelVelocityPassingMap.put(12.0, 105.0);

        turretDistanceToTimeOfFlightPassingMap = new InterpolatingDoubleTreeMap();
        turretDistanceToTimeOfFlightPassingMap.put(4.6, 0.75);
        turretDistanceToTimeOfFlightPassingMap.put(8.5, 0.928);

        currentIntakeState = IntakeConstants.INTAKE_DEPLOYED_BY_DEFAULT;

        flywheelSpeedFudgeFactor = FlywheelConstants.DEFAULT_FUDGE_FACTOR;
        hoodAngleFudgeFactor = HoodConstants.DEFAULT_FUDGE_FACTOR;

        loggingTable = NetworkTableInstance.getDefault().getTable("RobotState");
        robotPosePublisher = loggingTable.getStructTopic("Robot Odometry Pose", Pose2d.struct).publish();
        turretOdomPosePublisher = loggingTable.getStructTopic("Turret Odometry Pose", Pose2d.struct).publish();
        hoodFudgeFactorPublisher = loggingTable.getDoubleTopic("Current Hood Fudge Factor").publish();
        flywheelFudgeFactorPublisher = loggingTable.getDoubleTopic("Current Flywheel Fudge Factor").publish();
        robotVelocityOnFieldPublisher = loggingTable
                .getStructTopic("Robot Velocity Vector", Translation2d.struct).publish();
        robotVelocityAsVisionTargetPublisher = loggingTable
                .getStructTopic("Robot Velocity Vector Offset By Robot", Translation3d.struct).publish();
        turretRotationalVelocityOnFieldPublisher = loggingTable
                .getStructTopic("Turret Rotational Velocity Vector", Translation2d.struct).publish();
        turretRotationalVelocityAsVisionTargetPublisher = loggingTable
                .getStructTopic("Turret Rotational Velocity Vector Offset By Robot", Translation3d.struct).publish();

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

        Translation2d currentTurretTangentVelocity = getFieldRelativeTurretRotationalVelocityVector();
        turretRotationalVelocityOnFieldPublisher.accept(currentTurretTangentVelocity);
        turretRotationalVelocityAsVisionTargetPublisher.accept(
                new Translation3d(
                        currentTurretTangentVelocity.plus(getTurretOdomPose().getTranslation()))
                        .plus(new Translation3d(0.0, 0.0, 0.5)));
    }

    public void logAllDistances() {
        Translation2d robotCenter = this.getRobotOdomPose().getTranslation();
        Translation2d turretCenter = this.getTurretOdomPose().getTranslation();

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
    public double getHoodAngleFromPassingGoalPose(Pose2d poseToSetAngleFrom) {
        // Get distance between turret position and goal position, plug that into the
        // map
        return turretDistanceToHoodAnglePassingMap.get(
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
    public double getFlywheelVelocityFromPassingGoalPose(Pose2d poseToSetAngleFrom) {
        // Get distance between turret position and goal position, plug that into the
        // map
        return turretDistanceToFlywheelVelocityPassingMap.get(
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

    public Translation2d getFieldRelativeTurretRotationalVelocityVector() {
        double currentRobotAngularVelocity = robotChassisSpeedsSupplier.get().omegaRadiansPerSecond;

        // Turret position - robot position, resulting vector has magnitude of radius
        Translation2d turretToRobotCenterOnField = getTurretOdomPose().getTranslation()
                .minus(getRobotOdomPose().getTranslation());
        // Multiply by angular velocity in radians / sec to get resultant translational
        // velocity
        Translation2d tangentTurretVector = turretToRobotCenterOnField.rotateBy(Rotation2d.kCCW_90deg)
                .times(currentRobotAngularVelocity);

        return tangentTurretVector;
    }

    public ShootOnTheMoveOutputPoses calculateParametersForShootOnTheMove(Translation2d goalTranslationOnField) {

        ShootOnTheMoveOutputPoses outputData = new ShootOnTheMoveOutputPoses();

        Translation2d turretTranslationOnField = this.getTurretOdomPose().getTranslation();
        Translation2d turretVelocityOnFieldAsTranslation = this.getFieldRelativeRobotVelocityVector()
                .plus(this.getFieldRelativeTurretRotationalVelocityVector());

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
    public ShootOnTheMoveOutputPoses calculateParametersForPassOnTheMove(Translation2d goalTranslationOnField) {

        ShootOnTheMoveOutputPoses outputData = new ShootOnTheMoveOutputPoses();

        Translation2d turretTranslationOnField = this.getTurretOdomPose().getTranslation();
        Translation2d turretVelocityOnFieldAsTranslation = this.getFieldRelativeRobotVelocityVector()
                .plus(this.getFieldRelativeTurretRotationalVelocityVector());

        double displacement_x, displacement_y, distance_derived_from_current_time;

        double turret_velocity_x, turret_velocity_y;
        turret_velocity_x = turretVelocityOnFieldAsTranslation.getX();
        turret_velocity_y = turretVelocityOnFieldAsTranslation.getY();

        // Use Speed = Distance / Time
        double initial_distance_to_target = goalTranslationOnField.getDistance(turretTranslationOnField);
        double horizontal_projectile_velocity = initial_distance_to_target
                / turretDistanceToTimeOfFlightPassingMap.get(initial_distance_to_target);

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

            error = current_time_guess - turretDistanceToTimeOfFlightPassingMap.get(distance_derived_from_current_time);
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
