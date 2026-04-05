package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TurretConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommands {

    public static final double MAX_DRIVE_TRANSLATIONAL_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_DRIVE_ANGULAR_SPEED = RotationsPerSecond.of(1.0).in(RadiansPerSecond);

    public static final double DEADBAND_AS_DECIMAL = 0.01;
    public static final double CURVE_EXPONENT = 1.25;

    public static final double AUTO_AIM_TOLERANCE_DEGREES = 3;

    private static final SwerveRequest.FieldCentric teleopFieldCentricDriveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.FieldCentricFacingAngle teleopFieldCentricDriveRequestFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(7.0, 0.0, 0.0)
            .withCenterOfRotation(TurretConstants.ROBOT_TO_TURRET_CENTER.getTranslation());

    private static Translation2d modifyJoystickCurve(double inputX, double inputY) {
        Translation2d inputVector = new Translation2d(inputX, inputY);
        double inputMagnitude = inputVector.getNorm();
        Rotation2d inputAngle = inputVector.getAngle();

        double outputMagnitude = 0.0;
        if (inputMagnitude > DEADBAND_AS_DECIMAL)
            outputMagnitude = Math.pow(inputMagnitude - DEADBAND_AS_DECIMAL, CURVE_EXPONENT) /
                    Math.pow(1.0 - DEADBAND_AS_DECIMAL, CURVE_EXPONENT);

        return new Translation2d(outputMagnitude, inputAngle);
    }

    public static Command joystickDriveCommand(
            CommandSwerveDrivetrain drivetrain,
            CommandXboxController controller,
            double translationalScalar,
            double angularScalar) {
        return drivetrain.applyRequest(
                () -> {
                    Translation2d modifiedDriveInput = modifyJoystickCurve(
                            -controller.getLeftY(),
                            -controller.getLeftX());
                    Translation2d modifiedTurnInput = modifyJoystickCurve(
                            -controller.getRightX(),
                            0.0);

                    return teleopFieldCentricDriveRequest
                            .withCenterOfRotation(new Translation2d())
                            .withVelocityX(
                                    modifiedDriveInput.getX() * MAX_DRIVE_TRANSLATIONAL_SPEED * translationalScalar)
                            .withVelocityY(
                                    modifiedDriveInput.getY() * MAX_DRIVE_TRANSLATIONAL_SPEED * translationalScalar)
                            .withRotationalRate(
                                    modifiedTurnInput.getX() * MAX_DRIVE_ANGULAR_SPEED * angularScalar);
                });
    }

    public static Command joystickDriveWithCenterOfRotationCommand(
            CommandSwerveDrivetrain drivetrain,
            CommandXboxController controller,
            double translationalScalar,
            double angularScalar,
            Translation2d centerOfRotation) {
        return drivetrain.applyRequest(
                () -> {
                    Translation2d modifiedDriveInput = modifyJoystickCurve(
                            -controller.getLeftY(),
                            -controller.getLeftX());
                    Translation2d modifiedTurnInput = modifyJoystickCurve(
                            -controller.getRightX(),
                            0.0);

                    return teleopFieldCentricDriveRequest
                            .withCenterOfRotation(centerOfRotation)
                            .withVelocityX(
                                    modifiedDriveInput.getX() * MAX_DRIVE_TRANSLATIONAL_SPEED * translationalScalar)
                            .withVelocityY(
                                    modifiedDriveInput.getY() * MAX_DRIVE_TRANSLATIONAL_SPEED * translationalScalar)
                            .withRotationalRate(
                                    modifiedTurnInput.getX() * MAX_DRIVE_ANGULAR_SPEED * angularScalar);
                });
    }

}
