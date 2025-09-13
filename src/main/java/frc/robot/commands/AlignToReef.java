package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import org.littletonrobotics.junction.Logger;

public class AlignToReef extends Command {
    private CommandSwerveDrivetrain drivetrain;

  private final SwerveRequest.ApplyRobotSpeeds driveRequest =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
  // private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new
  // SwerveRequest.ApplyRobotSpeeds();

  private PIDController thetaController = new PIDController(1, 0, 0);
  private PIDController yController = new PIDController(1, 0, 0);
  private PIDController xController = new PIDController(1, 0, 0);

  private Pose2d targetPose;
  private double offset;
  private Rotation2d rotationOffset;

  public AlignToReef(
      CommandSwerveDrivetrain drive,
      double alignmentOffset,
      Pose2d alignmentPose,
      Rotation2d rotationOffset) {
    this.drivetrain = drive;
    this.offset = alignmentOffset;
    this.targetPose = alignmentPose;
    this.rotationOffset = rotationOffset;
  }

  @Override
  public void initialize() {
    Pose2d tCurrentPose = drivetrain.getState().Pose;

    Logger.recordOutput("/AutoAlign/CurrentPose", tCurrentPose);
    Logger.recordOutput("/AutoAlign/TargetPose", targetPose);
    Logger.recordOutput("/AutoAlign/Offset", offset);

    thetaController.setSetpoint(rotationOffset.getRadians());
    yController.setSetpoint(offset);
    thetaController.enableContinuousInput(0, 2 * Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(0.5));
    yController.setTolerance(Units.inchesToMeters(0.2));
  }

  @Override
  public void execute() {
    // double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
    Pose2d currentPose = drivetrain.getState().Pose;

    Transform2d offset = currentPose.minus(targetPose);

    double thetaVelocity = thetaController.calculate(offset.getRotation().getRadians(), 0);
    if (thetaController.atSetpoint()) {
      thetaVelocity = 0;
    }
    double yVelocity = yController.calculate(offset.getY());
    if (yController.atSetpoint()) {
      yVelocity = 0;
    }
    double xVelocity = xController.calculate(offset.getX());
    if (xController.atSetpoint()) {
      xVelocity = 0;
    }

    Rotation2d tagRotation = targetPose.getRotation();

    ChassisSpeeds driverCommandedVelocities =
        new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity);

    drivetrain.setControl(driveRequest.withSpeeds(driverCommandedVelocities));

    ChassisSpeeds fieldCommandedVelocities =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            driverCommandedVelocities, drivetrain.getOperatorForwardDirection());

    ChassisSpeeds tagRelativeCommandedVelocities =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldCommandedVelocities, tagRotation);

    tagRelativeCommandedVelocities.vyMetersPerSecond = yVelocity;
    tagRelativeCommandedVelocities.vxMetersPerSecond = xVelocity;
    tagRelativeCommandedVelocities.omegaRadiansPerSecond = thetaVelocity;

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(tagRelativeCommandedVelocities, tagRotation);

    //

  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}