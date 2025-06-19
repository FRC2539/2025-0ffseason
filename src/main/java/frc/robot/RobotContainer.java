// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N13;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.TunerConstants;
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



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final ThrustmasterJoystick rightJoystick = new ThrustmasterJoystick(0);
    private final ThrustmasterJoystick leftJoystick = new ThrustmasterJoystick(1);

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

        //auto = new Auto(drivetrain);

        configureBindings();

    }           

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-Math.pow(leftJoystick.getXAxis().getRaw(),3) * MaxSpeed) // Drive forward with negative Y (forward) POSSIBLY READD - TO FIX ANY INVERT ISSUES
                    .withVelocityY(Math.pow(leftJoystick.getYAxis().getRaw(), 3) * MaxSpeed) // Drive left with negative X (left)
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
        operatorController.getLeftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); 
        // operatorController.getX().whileTrue(placer.intake(12));
        // operatorController.getY().onTrue(elevator.setPosition(22.4));
        // operatorController.getA().onTrue(elevator.setPosition(.5));
        //operatorController.getY().onTrue(elevator.setVoltage(1.5).andThen(elevator.setPosition(1.5)));

        operatorController.getB().whileTrue(placer.intakeUntilPieceSet());
        // operatorContro
       // ller.getX().onTrue(climber.upPosition());
        // operatorController.getY().onTrue(climber.downPosition());

        drivetrain.registerTelemetry(logger::telemeterize);



        operatorController.getY().onTrue(modeManager.moveElevator(Position.L4));
        operatorController.getX().onTrue(modeManager.moveElevator(Position.L3));
        operatorController.getB().onTrue(modeManager.moveElevator(Position.L2));
        operatorController.getDPadDown().onTrue(modeManager.moveElevator(Position.L1));

        operatorController.getDPadLeft().onTrue(Commands.runOnce(() -> placer.setVoltage(0), placer));
        operatorController.getDPadRight().whileTrue(placer.intakeUntilPieceSet());
        operatorController.getA().onTrue(Commands.sequence(modeManager.moveElevator(Position.Home), placer.intakeUntilPieceSet()));
        operatorController.getRightTrigger().onTrue(placer.placePiece());





        // rightJoystick.getTrigger().onTrue(placer.runOnce(() -> placer.placePiece()));
        // operatorController.getDPadDown().whileTrue(placer.run(() -> placer.intake(2)));
        // operatorController.getDPadUp().whileTrue(placer.run(() -> placer.ejectReverse(2)));
        // operatorController.getDPadLeft().onTrue(placer.run(() -> placer.intakeUntilPieceContained()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
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
