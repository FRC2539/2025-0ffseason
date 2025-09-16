package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlignToReefCPPPID;
import frc.robot.commands.DriveDistance;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.modeManager.ModeManager.Position;

import java.io.IOException;
import java.util.Optional;


import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Auto {
    private final LoggedDashboardChooser<Command> autoChooser;
    private RobotConfig config; // PathPlanner robot configuration
    private RobotContainer container;



    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    // *NEW
    private final Field2d m_trajectoryField = new Field2d();

    public Auto(CommandSwerveDrivetrain drivetrain, RobotContainer ct) {
        this.container = ct;
        setUpPathPlanner(drivetrain);
        // named commands

        createNamedCommands();
        
        autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
        SmartDashboard.putData("Auto Path", m_trajectoryField);
        
    }

    public Command getAuto() {
        return autoChooser.get();
    }

    public void createNamedCommands() {

        Command driveToRightPlaceCommand = Commands.sequence(
           new AlignToReefCPPPID(container.drivetrain, -.184, -.05)
            ,
            new DriveDistance( // The name has been changed here
                this.container.drivetrain,
                -.5,
                180
            )
        );
     

        Command driveToLeftPlaceCommand = Commands.sequence(
           new AlignToReefCPPPID(container.drivetrain, .0884, -.071)
            ,
            new DriveDistance( // The name has been changed here
                this.container.drivetrain,
                -.5,
                180
            )
        );
       
        NamedCommands.registerCommand("place", container.placer.placePiece());
        NamedCommands.registerCommand("intake", container.placer.placePiece());
        
        NamedCommands.registerCommand("goto L4", container.modeManager.moveElevator(Position.L4));

        NamedCommands.registerCommand("align right", driveToRightPlaceCommand);
        NamedCommands.registerCommand("align left", driveToLeftPlaceCommand);
    }

    public void setUpPathPlanner(frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain drivetrain) {
        
            config = GlobalConstants.getRobotConfigPathplanner();
        
            
        AutoBuilder.configure(
                drivetrain::getRobotPose,
                drivetrain::resetPose,
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) ->
                        drivetrain.setControl(pathApplyRobotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController( // PPHolonomicController is the built in path
                        // following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                        ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drivetrain);

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> {
                    // Do whatever you want with the pose here
                    m_trajectoryField.getObject("Robot").setPose(pose);
                });
    }
}