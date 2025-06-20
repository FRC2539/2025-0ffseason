package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AlignToReefVision extends Command {
    private CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    double targetTx = -21.85;

    private int id;
    private PIDController thetaController = new PIDController(10, 0, 0);
    private PIDController yController = new PIDController(.1, 0, 0.005);
    //private VisionSubsystem camera;
    private DoubleSupplier xVelocity;
    public AlignToReefVision(CommandSwerveDrivetrain drivetrain,  boolean isLeftPole, DoubleSupplier xVelocity) {
        this.drivetrain = drivetrain;
        this.xVelocity = xVelocity;
        if (isLeftPole) {
            targetTx = 19.18;
        }
    }

    @Override
    public void initialize() {
        id = (int) LimelightHelpers.getFiducialID("limelight");
        double setpoint = 0;

        switch (id) {
            case 6:
                setpoint = 120;
                break;
            case 7:
                setpoint = 180;
                break;
            case 8:
                setpoint = -120;
                break;
            case 9:
                setpoint = -60;
                break;
            case 10:
                setpoint = 0;
                break;
            case 11:
                setpoint = 60;
                break;


            case 17:
                setpoint = 60;
                break;
            case 18:
                setpoint = 0;
                break;
            case 19:
                setpoint = -60; // -58?
                break;
            case 20:
                setpoint = -120;
                break;
            case 21:
                setpoint = 180;
                break;
            case 22:
                setpoint = 120;
                break;
                
                
            
        }
        //thetaController.setSetpoint(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(8).get().getRotation().getX());
        thetaController.setSetpoint(Units.degreesToRadians(setpoint));
        //thetaController.setSetpoint(0);
        yController.setSetpoint(targetTx);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);


        yController.setTolerance(0.2);
        thetaController.setTolerance(Units.degreesToRadians(1.5));
        //yController.setTolerance(0.5);

        
    }

    @Override
    public void execute() {
        double thetaVelocity = thetaController.calculate(drivetrain.getState().Pose.getRotation().getRadians());

        double yVelocity = yController.calculate(LimelightHelpers.getTX("limelight"));

    
        ChassisSpeeds velocityToApply = new ChassisSpeeds(xVelocity.getAsDouble(), yVelocity, thetaVelocity);
        // System.out.println(thetaController.atSetpoint());
        // if (thetaController.atSetpoint()) {
            //velocityToApply = new ChassisSpeeds(xVelocity.getAsDouble(), yVelocity, thetaVelocity);
        // } else {
        //     velocityToApply = new ChassisSpeeds(0, 0, thetaVelocity);
        // }

        //drivetrain.applyRequest(() -> applyRobotSpeeds.withSpeeds(velocityToApply));

        if (LimelightHelpers.getTV("limelight") == true) {
            drivetrain.setControl(applyRobotSpeeds.withSpeeds(velocityToApply));
        } else {
            drivetrain.setControl(applyRobotSpeeds.withSpeeds(new ChassisSpeeds(xVelocity.getAsDouble(), 0, thetaVelocity)));
        }
        
    }


    @Override
    public boolean isFinished() {
        
        return false; 
    }

}
