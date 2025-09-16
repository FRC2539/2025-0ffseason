package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class AlignToReefCPPPID extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.ApplyRobotSpeeds applySpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private final PIDController xController = new PIDController(1, 0, 0);
    private final PIDController yController = new PIDController(5, 0, 0);

    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            2.2,
            0.15,
            0.6,
            new TrapezoidProfile.Constraints(
                Math.toRadians(360), // Max velocity (radians per second)
                Math.toRadians(180)  // Max acceleration (radians per second squared)
            )
    );

    double targetTx = -.0821;
    double targetTy = -.0625;
    double desiredZ = 0;

    public AlignToReefCPPPID(CommandSwerveDrivetrain dt, double targetTx, double targetTy) {
        this.drivetrain = dt;
        addRequirements(drivetrain);
        this.targetTx = targetTx;
        this.targetTy = targetTy;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Math.toRadians(1));
    }

    @Override
    public void initialize() {
        xController.setSetpoint(this.targetTx);
        yController.setSetpoint(this.targetTy);
        thetaController.setGoal(desiredZ);

        xController.setTolerance(0.1);
        yController.setTolerance(0.01);
    }

    @Override
    public void execute() {
        String currentCamera = LimelightHelpers.getTV("limelight-left") ? "limelight-left" : "limelight-right";
        double[] botPose = LimelightHelpers.getTargetPose_CameraSpace(currentCamera);

        if (LimelightHelpers.getTV(currentCamera) && botPose.length > 0) {
            double currentX = botPose[0];
            double currentY = botPose[1];
            double currentAngle = Math.toRadians(botPose[5]);

            double xSpeed = 0;
            double ySpeed = 0;
            double thetaSpeed = 0;

            if ("limelight-right".equals(currentCamera)) {
                xSpeed = -xController.calculate(currentX);
                ySpeed = yController.calculate(currentY);
                thetaSpeed = thetaController.calculate(currentAngle, desiredZ + Math.toRadians(3.5));
            } else {
                xSpeed = xController.calculate(currentX);
                ySpeed = -yController.calculate(currentY);
                thetaSpeed = thetaController.calculate(currentAngle, desiredZ - Math.toRadians(3.5));
            }

            ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
            drivetrain.setControl(applySpeeds.withSpeeds(speeds));
        } else {
            drivetrain.setControl(applySpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
        }

        System.out.println("ALIGNING");
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}