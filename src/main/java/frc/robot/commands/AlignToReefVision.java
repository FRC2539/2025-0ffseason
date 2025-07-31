package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class AlignToReefVision extends Command {
    private CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    double targetTx = 0;//-19.8; //-21.85

    private int id;
    private PIDController thetaController = new PIDController(.1, 0, 0.005);
    private PIDController yController = new PIDController(.1, 0, 0.005);
    //private VisionSubsystem camera;
    private DoubleSupplier xVelocity;
    //private DoubleSupplier yVelocity;
    public AlignToReefVision(CommandSwerveDrivetrain drivetrain,  boolean isLeftPole, DoubleSupplier xVelocity) {
        this.drivetrain = drivetrain;
        this.xVelocity = xVelocity;

        targetTx = 0;
    }

    @Override
    public void initialize() {

        yController.setTolerance(1.5);
        thetaController.setTolerance(Units.degreesToRadians(1.5));
        
    }

    @Override
    public void execute() {

        if (LimelightHelpers.getTV("limelight") == true) {
            id = (int) LimelightHelpers.getFiducialID("limelight");
        }
        
        double setpoint = 0;
        
        switch (id) {
            case 6:
                setpoint = -120;
                break;
            case 7:
                setpoint = 180;
                break;
            case 8:
                setpoint = 120;
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

        //thetaController.setSetpoint(Units.degreesToRadians(setpoint));


        double thetaVelocity = thetaController.calculate(-drivetrain.getState().Pose.getRotation().getRadians(), Units.degreesToRadians(setpoint));

        double yVelocity = yController.calculate(LimelightHelpers.getTX("limelight"),0);

        System.out.println("ty: "+yVelocity+" c: "+ LimelightHelpers.getTX("limelight")+" t:"+0);
        System.out.println("tz: "+thetaVelocity+" c: "+ -drivetrain.getState().Pose.getRotation().getDegrees()+" t:"+setpoint);
    
        ChassisSpeeds velocityToApply = new ChassisSpeeds(xVelocity.getAsDouble(), yVelocity, thetaVelocity);
        
        if (LimelightHelpers.getTV("limelight") == true) {
            drivetrain.setControl(applyRobotSpeeds.withSpeeds(velocityToApply));
        }
                
    }


    @Override
    public boolean isFinished() {
        if (yController.atSetpoint() && thetaController.atSetpoint()){
            System.out.println("finished");
        } else {
            System.out.println("not finished y: "+yController.getError()+" y tol: "+yController.getErrorTolerance()+" z: "+thetaController.getError()+" z tol: "+thetaController.getErrorTolerance());
        }
        return yController.atSetpoint() && thetaController.atSetpoint();
        // return false; 
    }

}
