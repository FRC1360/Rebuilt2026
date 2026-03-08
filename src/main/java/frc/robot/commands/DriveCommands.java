package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.RobotState;

public class DriveCommands {

    public static final double MAX_DRIVE_TRANSLATIONAL_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_DRIVE_ANGULAR_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public static final double DEADBAND_AS_DECIMAL = 0.15;

    public static final double AUTO_AIM_TOLERANCE_DEGREES = 3;

    private static final SwerveRequest.FieldCentric teleopFieldCentricDriveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.FieldCentricFacingAngle teleopFieldCentricDriveRequestFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(7.0, 0.0, 0.0)
            .withCenterOfRotation(TurretConstants.ROBOT_TO_TURRET_CENTER.getTranslation());

    private static double modifyJoystickCurve(double input) {
        double factor = 0.6;
        return (factor * Math.pow(input, 3)) + ((1.0 - factor) * input);
    }

    public static Command joystickDriveCommand(
            CommandSwerveDrivetrain drivetrain,
            CommandXboxController controller,
            double translationalScalar,
            double angularScalar) {
        return drivetrain.applyRequest(
                () -> teleopFieldCentricDriveRequest
                        .withDeadband(
                                MAX_DRIVE_TRANSLATIONAL_SPEED * DEADBAND_AS_DECIMAL * translationalScalar)
                        .withRotationalDeadband(
                                MAX_DRIVE_ANGULAR_SPEED * DEADBAND_AS_DECIMAL * angularScalar)
                        .withVelocityX(
                                modifyJoystickCurve(-controller.getLeftY()) * MAX_DRIVE_TRANSLATIONAL_SPEED
                                        * translationalScalar)
                        .withVelocityY(
                                modifyJoystickCurve(-controller.getLeftX()) * MAX_DRIVE_TRANSLATIONAL_SPEED
                                        * translationalScalar)
                        .withRotationalRate(
                                modifyJoystickCurve(-controller.getRightX()) * MAX_DRIVE_ANGULAR_SPEED
                                        * angularScalar));
    }

    public static Command joystickDriveFacingAngleCommand(
            CommandSwerveDrivetrain drivetrain,
            CommandXboxController controller,
            double translationalScalar,
            Rotation2d fieldRelativeAngle) {

        teleopFieldCentricDriveRequestFacingAngle.HeadingController.setTolerance(
                Units.degreesToRadians(AUTO_AIM_TOLERANCE_DEGREES));

        return drivetrain.applyRequest(
                () -> teleopFieldCentricDriveRequestFacingAngle
                        .withDeadband(
                                MAX_DRIVE_TRANSLATIONAL_SPEED * DEADBAND_AS_DECIMAL * translationalScalar)
                        .withVelocityX(
                                modifyJoystickCurve(-controller.getLeftY()) * MAX_DRIVE_TRANSLATIONAL_SPEED
                                        * translationalScalar)
                        .withVelocityY(
                                modifyJoystickCurve(-controller.getLeftX()) * MAX_DRIVE_TRANSLATIONAL_SPEED
                                        * translationalScalar)
                        .withTargetDirection(fieldRelativeAngle));
    }

    public static Command joystickDriveFacingPoseCommand(
            CommandSwerveDrivetrain drivetrain,
            CommandXboxController controller,
            double translationalScalar,
            Pose2d goalPose,
            Rotation2d rotationOffset) {

        teleopFieldCentricDriveRequestFacingAngle.HeadingController.setTolerance(
                Units.degreesToRadians(AUTO_AIM_TOLERANCE_DEGREES));

        RobotState robotState = RobotState.getInstance();

        return drivetrain.applyRequest(
                () -> teleopFieldCentricDriveRequestFacingAngle
                        .withDeadband(
                                MAX_DRIVE_TRANSLATIONAL_SPEED * DEADBAND_AS_DECIMAL * translationalScalar)
                        .withVelocityX(
                                modifyJoystickCurve(-controller.getLeftY()) * MAX_DRIVE_TRANSLATIONAL_SPEED
                                        * translationalScalar)
                        .withVelocityY(
                                modifyJoystickCurve(-controller.getLeftX()) * MAX_DRIVE_TRANSLATIONAL_SPEED
                                        * translationalScalar)
                        .withTargetDirection(PhotonUtils.getYawToPose(
                                new Pose2d(robotState.getTurretOdomPose().getTranslation(), new Rotation2d()),
                                goalPose).plus(rotationOffset)));
    }

    public static Trigger headingControllerAtTarget = new Trigger(
            () -> teleopFieldCentricDriveRequestFacingAngle.HeadingController.atSetpoint());

}
