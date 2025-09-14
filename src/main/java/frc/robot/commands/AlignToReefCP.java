package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;


public class AlignToReefCP extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.ApplyRobotSpeeds applySpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private final PIDController xController = new PIDController(1, 0, 0); //3
    private final PIDController yController = new PIDController(5, 0, 0); //5
    private final PIDController thetaController = new PIDController(10, 0, 0.01); //10

    double targetTx = -.0821;
    double targetTy = -.0625;
    double desiredZ = 0;

    public AlignToReefCP(CommandSwerveDrivetrain dt,double targetTx, double targetTy) {
        this.drivetrain = dt;
        addRequirements(drivetrain);
        this.targetTx = targetTx;
        this.targetTy = targetTy;
    }

    @Override
    public void initialize() {
        xController.setSetpoint(this.targetTx);
        yController.setSetpoint(this.targetTy);
        thetaController.setSetpoint(desiredZ);

        xController.setTolerance(0.1); //meters
        yController.setTolerance(0.01); //meters
        thetaController.setTolerance(.1); // Radians
    }

    @Override
    public void execute() {
        String currentCamera = LimelightHelpers.getTV("limelight-left") ? "limelight-left" : "limelight-right";
        double[] botPose = LimelightHelpers.getTargetPose_CameraSpace(currentCamera);

        
        if (LimelightHelpers.getTV(currentCamera)) {
            double currentX = botPose[0];
            double currentY = botPose[1];
            double currentAngle = botPose[5];

            double xSpeed = 0;
                double ySpeed =0;
                double thetaSpeed = 0;

            if (currentCamera == "limelight-right"){
                xSpeed = -xController.calculate(currentX);
                ySpeed = yController.calculate(currentY);
                thetaSpeed = thetaController.calculate(Math.toRadians(currentAngle), desiredZ + Math.toRadians(3.5));
            }else{
                xSpeed = xController.calculate(currentX);
                ySpeed = -yController.calculate(currentY);
                thetaSpeed = thetaController.calculate(Math.toRadians(currentAngle), desiredZ - Math.toRadians(3.5));
                
            }          

            //System.out.println("x current: " + currentX +" y current: " + currentY + " angle: "+ currentAngle);
            
            ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
            drivetrain.setControl(applySpeeds.withSpeeds(speeds));            
        } else {
            drivetrain.setControl(applySpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
        }
    }

    @Override
    public boolean isFinished() {
        if (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()){
        // if (xController.atSetpoint()){
            System.out.println("finished");
        } else{
            System.out.println("not finished");
        }
        
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}