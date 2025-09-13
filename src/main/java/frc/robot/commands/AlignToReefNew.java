package frc.robot.commands;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class AlignToReefNew extends Command {
    
    private final SwerveRequest.ApplyRobotSpeeds applySpeeds = new SwerveRequest.ApplyRobotSpeeds();

    CommandSwerveDrivetrain drivetrain;
    double targetTx;
    double targetTa;
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1.4328764321, 0, 0);
    PIDController thetaController = new PIDController(2.2, 0.15, 0.6);

    public AlignToReefNew(CommandSwerveDrivetrain dt, double targetTx, double targetTa) {
        this.drivetrain = dt;
        this.targetTx = targetTx;
        this.targetTa = targetTa;
    }

    @Override
    public void initialize() {
        xController.setSetpoint(0.3114);
        yController.setSetpoint(0);

        yController.setTolerance(0.0762);
        thetaController.setSetpoint(0);
        thetaController.setTolerance(0.08);

    }

    @Override
    public void execute() {
        String currentCamera = LimelightHelpers.getTV("limelight-left") ? "limelight-left" : "limelight-right";

        double[] positions = LimelightHelpers.getBotPose_TargetSpace(currentCamera);

        ChassisSpeeds speeds = new ChassisSpeeds();

        
        if (true) {
            speeds.vxMetersPerSecond = -xController.calculate(positions[1], 0.3114);
            speeds.vyMetersPerSecond = -yController.calculate(positions[0], targetTx);
        }
        
        if (LimelightHelpers.getTV(currentCamera)) {
            //speeds.omegaRadiansPerSecond = -thetaController.calculate(Math.toRadians(positions[4]));
        }
        
        //System.out.println(speeds.omegaRadiansPerSecond);
        
        System.out.println(xController.atSetpoint() + " " + positions[1]);
        
        drivetrain.setControl(applySpeeds.withSpeeds(speeds));
    }
}
