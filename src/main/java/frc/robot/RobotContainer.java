// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOKraken;


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



    public RobotContainer() {
        
        if(Robot.isReal()){;
            elevator = new ElevatorSubsystem(new ElevatorIOKraken());
        }
        else {
            elevator = new ElevatorSubsystem(new ElevatorIOSim());
        }

        configureBindings();

    }           

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-Math.pow(leftJoystick.getXAxis().getRaw(),3) * MaxSpeed * 0.3) // Drive forward with negative Y (forward) POSSIBLY READD - TO FIX ANY INVERT ISSUES
                    .withVelocityY(Math.pow(leftJoystick.getYAxis().getRaw(), 3) * MaxSpeed * 0.3) // Drive left with negative X (left)
                    .withRotationalRate(Math.pow(-rightJoystick.getXAxis().getRaw(), 3) * MaxAngularRate * 0.3).withDeadband(0.02) // Drive counterclockwise with negative X (left)
            )
        );

        rightJoystick.getRightBottomLeft().whileTrue(drivetrain.applyRequest(() -> brake));
        rightJoystick.getRightBottomRight().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-rightJoystick.getYAxis().getRaw(), -rightJoystick.getXAxis().getRaw()))
        ));
        

        // Run SysId routines when holding back/start and X/Y. 
        // Note that each routine should be run exactly once in a single log.
        rightJoystick.getTrigger().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        rightJoystick.getTrigger().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // rightJoystick.getTrigger().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // rightJoystick.getTrigger().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); stupid lets do this later

        // reset the field-centric heading on left bumper press 
        operatorController.getLeftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); 

        // operatorController.getX().onTrue(climber.upPosition());
        // operatorController.getY().onTrue(climber.downPosition());


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
