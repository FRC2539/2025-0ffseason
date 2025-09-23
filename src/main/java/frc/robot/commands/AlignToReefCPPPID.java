package frc.robot.commands;

import java.util.Timer;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class AlignToReefCPPPID extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.ApplyRobotSpeeds applySpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private final PIDController xController = new PIDController(1, 0.0001, 0);
    private final PIDController yController = new PIDController(.05, 0, 0.001);
    private final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Math.toRadians(360), // Max velocity (radians per second)
                Math.toRadians(180)  // Max acceleration (radians per second squared)
            )
    );

    double targetTx = -7;
    double targetTy = .05;
    double desiredZ = 0.4;

    String c;
    public 
    AlignToReefCPPPID(CommandSwerveDrivetrain dt, double targetTx, double targetTy, String camera) {
        this.drivetrain = dt;
        addRequirements(drivetrain);
        this.targetTx = targetTx;
        this.targetTy = targetTy;
        this.c = camera;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Math.toRadians(1));
    }

    @Override
    public void initialize() {
        xController.setSetpoint(this.targetTx);
        yController.setSetpoint(this.targetTy);
        thetaController.setGoal(desiredZ);

        xController.setTolerance(0.025);
        yController.setTolerance(0.5);
    }

    @Override
    public void execute() {
       // String currentCamera = LimelightHelpers.getTV("limelight-left") ? "limelight-left" : "limelight-right";
        String currentCamera = c;
        double[] botPose = LimelightHelpers.getTargetPose_CameraSpace(currentCamera);

        if (LimelightHelpers.getTV(currentCamera) && botPose.length > 0) {
            double currentX = botPose[0];
            //double currentY = botPose[1];
            double currentY = LimelightHelpers.getTX(currentCamera);
            double currentAngle = botPose[5];

            double xSpeed = 0;
            double ySpeed = 0;
            double thetaSpeed = 0;

            if ("limelight-right".equals(currentCamera)) {
                xSpeed = -xController.calculate(currentX);
                ///ySpeed = yController.calculate(currentY);
                thetaSpeed = thetaController.calculate(currentAngle, desiredZ);
            } else {
                xSpeed = xController.calculate(currentX);
                //ySpeed = yController.calculate(currentY);
                thetaSpeed = thetaController.calculate(currentAngle, desiredZ);
            }


            if (xController.atSetpoint() && thetaController.atGoal()) {
                ySpeed = yController.calculate(currentY);
            } else {
                timer.start();

                if (timer.get() < 0.5) {
                    ySpeed =  yController.calculate(currentY);
                }
                
            }

           

            ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, -thetaSpeed);
            drivetrain.setControl(applySpeeds.withSpeeds(speeds));
        } else {
            drivetrain.setControl(applySpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
        }

        System.out.println(isFinished());
    }

    @Override
    public boolean isFinished() {
        //System.out.println(String.valueOf(yController.atSetpoint()) + yController.getError());
        System.out.println(yController.atSetpoint() + "   " + xController.atSetpoint() + "   " + thetaController.atSetpoint());
        return yController.atSetpoint() && thetaController.atGoal() && xController.atSetpoint();
        
        
    }
}