// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N13;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlignAndDriveToReef;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.AlignToReefVision;
import frc.robot.commands.DriveDistance;
import frc.robot.constants.AlignConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.modeManager.ModeManager;
import frc.robot.subsystems.modeManager.ModeManager.Position;
import frc.robot.subsystems.placer.PlacerIOSRX;
import frc.robot.subsystems.placer.PlacerIOSim;
import frc.robot.subsystems.placer.PlacerSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;
//import frc.robot.subsystems.elevator.ElevatorIOTalonFX;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final ThrustmasterJoystick rightJoystick = new ThrustmasterJoystick(1);
    private final ThrustmasterJoystick leftJoystick = new ThrustmasterJoystick(0);

    private final LogitechController operatorController = new LogitechController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator;
    public final PlacerSubsystem placer;

    //public final Auto auto;
    public final ModeManager modeManager;

    //public final VisionSubsystem camera; 

    public RobotContainer() {
        
        if(Robot.isReal()){;
            elevator = new ElevatorSubsystem(new ElevatorIOTalonFX());
            placer = new PlacerSubsystem(new PlacerIOSRX());
            modeManager = new ModeManager(elevator, placer);
            //camera = new VisionSubsystem((Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) -> {
        //         drivetrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        //     }, new VisionIOLimelight("limelight", () -> drivetrain.getPigeon2().getRotation2d()));
        }
        else {
            elevator = new ElevatorSubsystem(new ElevatorIOSim());
            placer = new PlacerSubsystem(new PlacerIOSim());
            modeManager = null;
            //camera = null;
        }

        //auto = new Auto(drivetrain, this);

        configureBindings();

    }           

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityY(-Math.pow(leftJoystick.getXAxis().getRaw(),3) * MaxSpeed) // Drive forward with negative Y (forward) POSSIBLY READD - TO FIX ANY INVERT ISSUES
                    .withVelocityX(-Math.pow(leftJoystick.getYAxis().getRaw(), 3) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(Math.pow(-rightJoystick.getXAxis().getRaw(), 3) * MaxAngularRate).withDeadband(0.02) // Drive counterclockwise with negative X (left)
            )
        );

        rightJoystick.getRightBottomLeft().whileTrue(drivetrain.applyRequest(() -> brake));
        rightJoystick.getRightBottomRight().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-rightJoystick.getYAxis().getRaw(), -rightJoystick.getXAxis().getRaw()))
        ));
        

        // Run SysId routines when holding back/start and X/Y. 
        // Note that each routine should be run exactly once in a single log.
        
        // rightJoystick.getTrigger().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // rightJoystick.getTrigger().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); stupid lets do this later

        // reset the field-centric heading on left bumper press 
        //operatorController.getLeftBumper().onTrue(elevator.setVoltage(0));
        operatorController.getRightBumper().onTrue(Commands.runOnce(() -> drivetrain.resetPose(new Pose2d(0,0, drivetrain.getOperatorForwardDirection()))));
        // operatorController.getX().whileTrue(placer.intake(12));
        // operatorController.getY().onTrue(elevator.setPosition(22.5));
        // operatorController.getX().onTrue(elevator.setPosition(8));
        // operatorController.getA().onTrue(elevator.setPosition(0));

        operatorController.getA().onTrue(Commands.parallel(modeManager.moveElevator(Position.Home), placer.intakeUntilPieceSet()));
        operatorController.getB().onTrue(modeManager.moveElevator(Position.L2));
        operatorController.getX().onTrue(modeManager.moveElevator(Position.L3));
        operatorController.getY().onTrue(modeManager.moveElevator(Position.L4));
        
        leftJoystick
        .getBottomThumb()
        .whileTrue(alignToReef(AlignConstants.leftOffset));
    rightJoystick
        .getBottomThumb()
        .whileTrue(alignToReef(AlignConstants.rightOffset));
    

        operatorController.getDPadUp().onTrue(placer.setVoltage(0));
        operatorController.getDPadDown().onTrue(modeManager.moveElevator(Position.L1));
        operatorController.getDPadLeft().onTrue(placer.placePiece());
        operatorController.getDPadRight().onTrue(placer.intakeUntilPieceSet());
        
        

        

        // operatorController.getLeftTrigger().whileTrue(elevator.setVoltage(12));
        // operatorController.getRightTrigger().whileTrue(elevator.setVoltage(-12));

        //operatorController.getRightTrigger().onTrue(new AlignToReefVision(drivetrain, false, () -> {return -Math.pow(leftJoystick.getYAxis().getRaw(), 3) * MaxSpeed;}));
        //operatorController.getLeftTrigger().onTrue(new AlignToReefVision(drivetrain, true, () -> {return -Math.pow(leftJoystick.getYAxis().getRaw(), 3) * MaxSpeed;}));
        //operatorController.getB().whileTrue(placer.intakeUntilPieceSet());
        // operatorController.getX().onTrue(climber.upPosition());
        // operatorController.getY().onTrue(climber.downPosition());

        //rightJoystick.getRightThumb().whileTrue(new AlignToReefVision(drivetrain, false, () -> {return -Math.pow(leftJoystick.getYAxis().getRaw(), 3) * MaxSpeed;}));
        //.getLeftThumb().whileTrue(new AlignToReefVision(drivetrain, true, () -> {return -Math.pow(leftJoystick.getYAxis().getRaw(), 3) * MaxSpeed;}));
        // Command driveToRightPlaceCommand = Commands.sequence(
        //    new AlignToReefVision(drivetrain, false, () -> {return -Math.pow(leftJoystick.getYAxis().getRaw(), 3) * MaxSpeed;})
        //     ,
        //     new DriveDistance( // The name has been changed here
        //         drivetrain,
        //         -.21,
        //         90
        //     )
        // );
        // rightJoystick.getRightThumb().whileTrue(driveToRightPlaceCommand);

        // Command driveToLeftPlaceCommand = Commands.sequence(
        //    new AlignToReefVision(drivetrain, false, () -> {return -Math.pow(leftJoystick.getYAxis().getRaw(), 3) * .6;})//maxspeed
        //     ,
        //     new DriveDistance( // The name has been changed here
        //         drivetrain,
        //         -.21,
        //         -90
        //     )
        // );
        // rightJoystick.getLeftThumb().whileTrue(driveToLeftPlaceCommand);

        // // Create the sequential command group: Align then Drive
        // Command driveToPlaceCommand = Commands.sequence(
        //     new AlignToAprilTagRelative(
        //         drivetrain,
        //         0,
        //         0,
        //         0
        //     )
        //     // ,
        //     // new DriveDistance( // The name has been changed here
        //     //     drivetrain,
        //     //     -.20,
        //     //     -90
        //     // )
        // );
        
        // rightJoystick.getLeftThumb().whileTrue(driveToPlaceCommand);


        drivetrain.registerTelemetry(logger::telemeterize);

    
        rightJoystick.getTrigger().onTrue(placer.placePiece());






        // rightJoystick.getTrigger().onTrue(placer.runOnce(() -> placer.placePiece()));
        // operatorController.getDPadDown().whileTrue(placer.run(() -> placer.intake(2)));
        // operatorController.getDPadUp().whileTrue(placer.run(() -> placer.ejectReverse(2)));
        // operatorController.getDPadLeft().onTrue(placer.run(() -> placer.intakeUntilPieceContained()));
    }

    public Command getAutonomousCommand() {
        return Commands.none(); //auto.getAuto();
    }

    public Command alignToReef(int tag, double offset, Rotation2d rotOffset) {
    Pose2d alignmentPose =
        VisionConstants.aprilTagLayout
            .getTagPose(tag)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(new Translation2d(AlignConstants.reefDistance, offset), rotOffset));
    return new AlignToReef(drivetrain, offset, alignmentPose, Rotation2d.kPi);
  }

  public Command alignToReef(int tag, double offset) {
    return alignToReef(tag, offset, Rotation2d.kZero);
  }

  public Command alignToReef(double offset) {
    return Commands.defer(
        () -> {
          Pose2d alignmentPose = drivetrain.findNearestAprilTagPose();
          return new AlignToReef(drivetrain, offset, alignmentPose, Rotation2d.kPi);
        },
        Set.of(drivetrain));
  }

  public Command alignAndDriveToReef(int tag, double offset) {
    Pose2d alignmentPose =
        VisionConstants.aprilTagLayout
            .getTagPose(tag)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    new Translation2d(AlignConstants.reefDistance, offset), new Rotation2d()));
    return new AlignAndDriveToReef(drivetrain, 0, alignmentPose, Rotation2d.kPi);
  }


    // public Command alignToReef(int tag, double offset, Rotation2d rotOffset) {
    //     Pose2d alignmentPose =
    //             VisionConstants.aprilTagLayout
    //                     .getTagPose(tag)
    //                     .get()
    //                     .toPose2d()
    //                     .plus(
    //                             new Transform2d(
    //                                     new Translation2d(AligningConstants.reefDistance, offset),
    //                                     rotOffset));
    //     return new AlignToReef(
    //             drivetrain,
    //             leftJoystickVelocityX,
    //             leftJoystickVelocityY,
    //             0,
    //             alignmentPose,
    //             Rotation2d.kPi);
    // }

    // public Command alignToReef(int tag, double offset) {
    //     return alignToReef(tag, offset, Rotation2d.kZero);
    // }

    // // Automatically chooses closest tag
    // public Command alignToReef(double offset) {
    //     return Commands.defer(
    //             () -> {
    //                 Pose2d alignmentPose = drivetrain.findNearestAprilTagPose();
    //                 return new AlignToReef(
    //                         drivetrain,
    //                         leftJoystickVelocityX,
    //                         leftJoystickVelocityY,
    //                         offset,
    //                         alignmentPose,
    //                         Rotation2d.kPi);
    //             },
    //             Set.of(drivetrain));
    // }

    // public Command alignAndDriveToReef(int tag, double offset) {
    //     Pose2d alignmentPose =
    //             VisionConstants.aprilTagLayout
    //                     .getTagPose(tag)
    //                     .get()
    //                     .toPose2d()
    //                     .plus(
    //                             new Transform2d(
    //                                     new Translation2d(AligningConstants.reefDistance, offset),
    //                                     new Rotation2d()));
    //     return new AlignAndDriveToReef(drivetrain, 0, alignmentPose, Rotation2d.kPi);
    // }

}
