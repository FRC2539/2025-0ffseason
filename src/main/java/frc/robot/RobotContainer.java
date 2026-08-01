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
import frc.robot.commands.AlignToReefCPPPID;
import frc.robot.commands.AlignToReefVision;
import frc.robot.commands.DriveDistance;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

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

   // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final ThrustmasterJoystick rightJoystick = new ThrustmasterJoystick(1);
    private final ThrustmasterJoystick leftJoystick = new ThrustmasterJoystick(0);

    //private final LogitechController operatorController = new LogitechController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Auto auto;


    public RobotContainer() {
        
        if(Robot.isReal()){;      
            // camera = new VisionSubsystem((Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) -> {
            //     drivetrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
            // }, new VisionIOLimelight("limelight-left", () -> drivetrain.getPigeon2().getRotation2d()), new VisionIOLimelight("limelight-right", () -> drivetrain.getPigeon2().getRotation2d()));
        }
        else {
        }

        auto = new Auto(drivetrain, this);

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

        


    
        //atorController.getLeftTrigger().whileTrue(elevator.setVoltage(12));
        // operatorController.getRightTrigger().whileTrue(elevator.setVoltage(-12));

        //operatorController.getRightTrigger().onTrue(new AlignToReefVision(drivetrain, false, () -> {return -Math.pow(leftJoystick.getYAxis().getRaw(), 3) * MaxSpeed;}));
        //operatorController.getLeftTrigger().onTrue(new AlignToReefVision(drivetrain, true, () -> {return -Math.pow(leftJoystick.getYAxis().getRaw(), 3) * MaxSpeed;}));
        //operatorController.getB().whileTrue(placer.intakeUntilPieceSet());
        // operatorController.getX().onTrue(climber.upPosition());
        // operatorController.getY().onTrue(climber.downPosition());






        // rightJoystick.getTrigger().onTrue(placer.runOnce(() -> placer.placePiece()));
        // operatorController.getDPadDown().whileTrue(placer.run(() -> placer.intake(2)));
        // operatorController.getDPadUp().whileTrue(placer.run(() -> placer.ejectReverse(2)));
        // operatorController.getDPadLeft().onTrue(placer.run(() -> placer.intakeUntilPieceContained()));
    }

    public Command getAutonomousCommand() {
        return auto.getAuto();
    }


}
