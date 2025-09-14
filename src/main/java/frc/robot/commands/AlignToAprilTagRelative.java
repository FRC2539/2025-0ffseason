package frc.robot.commands;

//import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

//import frc.robot.commands.LimelightHelpers;

public class AlignToAprilTagRelative extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    // PID gains
    private PIDController xController = new PIDController(1.5, 0.0, .0001);
    private PIDController yController = new PIDController(.1, 0.0, 0.01);    
    private PIDController thetaController = new PIDController(60, 0.0, 0.00001); //p60

    private double desiredTargetX_robotRelative;
    private double desiredTargetY_robotRelative;
    private double desiredTargetRotationZ_radians;

    public AlignToAprilTagRelative(CommandSwerveDrivetrain drivetrain,
                                   double desiredTargetX_robotRelative, double desiredTargetY_robotRelative,
                                   double desiredRobotRotation_relativeToTag) {
        this.drivetrain = drivetrain;
        this.desiredTargetX_robotRelative = desiredTargetX_robotRelative;
        this.desiredTargetY_robotRelative = desiredTargetY_robotRelative;
        // Convert desired robot rotation to the target's rotation relative to the robot.
        this.desiredTargetRotationZ_radians = Units.degreesToRadians(-desiredRobotRotation_relativeToTag);

        addRequirements(drivetrain);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        xController.setTolerance(0.05);
        yController.setTolerance(0.3);
        thetaController.setTolerance(Units.degreesToRadians(.5));
    }

    // Simpler constructor for aligning directly in front with no lateral/rotational offset
    public AlignToAprilTagRelative(CommandSwerveDrivetrain drivetrain, double desiredTargetX_robotRelative) {
        this(drivetrain, desiredTargetX_robotRelative, 0.0, 0.0);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();
        
        // Ensure vision processing is enabled on Limelight (e.g., pipeline 0 for AprilTags)
        LimelightHelpers.setPipelineIndex("limelight", 0); 
    }

    @Override
    public void execute() {
        // Only proceed if Limelight has *any* valid target
        if (LimelightHelpers.getTV("limelight")) {
            // Get the pose of the target relative to the robot's camera.
            // This will automatically be the primary target Limelight is tracking.
            // targetPose_robotRelative = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
            Pose3d targetPose_robotRelative = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
            //double t2_TY = LimelightHelpers.getTY("limelight");
            System.out.println("Limelight detected a tag!");

            // Check if the Pose3d is valid (not all zeros or null)
            if (targetPose_robotRelative != null && targetPose_robotRelative.getTranslation().getNorm() > 0.001) {
                double currentTargetX = targetPose_robotRelative.getX();                
                double currentTargetY = targetPose_robotRelative.getY();
                double currentTargetRotationZ = targetPose_robotRelative.getRotation().getZ();

                //System.out.println("Limelight TargetX: "+currentTargetX+" Y:"+currentTargetY+" Z: "+currentTargetRotationZ);
                //System.out.println("Limelight Z: "+currentTargetRotationZ);

                //double xVelocity = xController.calculate(currentTargetX, desiredTargetX_robotRelative);
                double xVelocity = xController.calculate(-currentTargetX, desiredTargetX_robotRelative);
                double yVelocity = yController.calculate(-currentTargetY, desiredTargetY_robotRelative);
                double thetaVelocity = thetaController.calculate(currentTargetRotationZ, desiredTargetRotationZ_radians)
                ;
                //System.out.println("tx: "+xVelocity+" c: "+ currentTargetX+" t:"+desiredTargetX_robotRelative);
                //System.out.println("tx: "+thetaVelocity+" c: "+ currentTargetRotationZ+" t:"+desiredTargetRotationZ_radians);
                //System.out.println("tcZ: "+thetaVelocity);
                //System.out.println("tcZ: "+thetaVelocity);


                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, xVelocity, thetaVelocity);;
                // if (!xController.atSetpoint()){
                //     chassisSpeeds = new ChassisSpeeds(0, xVelocity, 0);
                // } else if(!thetaController.atSetpoint()){
                //     chassisSpeeds = new ChassisSpeeds(0, xVelocity, thetaVelocity);
                // } else if(!yController.atSetpoint()){
                //     //chassisSpeeds = new ChassisSpeeds(yVelocity, 0, 0);
                //}
                
                //ChassisSpeeds chassisSpeeds = new ChassisSpeeds(yVelocity,0, 0);
               
                //System.out.println("x: "+xVelocity+" y:"+yVelocity+" t:"+thetaVelocity);


                drivetrain.setControl(applyRobotSpeeds.withSpeeds(chassisSpeeds));
            } else {
                // Limeli\][ght saw a target, but the pose estimate was invalid (e.g., too far, bad solve)
                drivetrain.setControl(applyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
                System.out.println("Limelight detected a tag, but pose estimate is invalid!");
            }
        } else {
            // Limelight doesn't detect any AprilTag
            drivetrain.setControl(applyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
            System.out.println("No AprilTag detected by Limelight!");
        }
    }

    @Override
    public boolean isFinished() {
        //System.out.println("Finished? "+xController.atSetpoint()+" setPoint:"+xController.getSetpoint()+"t:"+xController.getErrorTolerance());
        //return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
        //Pose3d targetPose_robotRelative = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
        //double currentTargetX = targetPose_robotRelative.getX();   
        //double currentTargetRotationZ = targetPose_robotRelative.getRotation().getZ();
        // if (currentTargetRotationZ < .5){ //currentTargetX < .05
        //     System.out.println("finished");
        //     return true;            
        // } else {
        //     System.out.println("not finished");
        //     return false;
        // }

        //if (xController.atSetpoint() && thetaController.atSetpoint() && yController.atSetpoint()){
        if (xController.atSetpoint() && thetaController.atSetpoint()){
            System.out.println("finished");
        } else {
            System.out.println("not finished");
        }
        return xController.atSetpoint() && thetaController.atSetpoint();
        //return xController.atSetpoint() && thetaController.atSetpoint()&& yController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(applyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
        System.out.println("AlignToAprilTagRelative command finished or interrupted.");
    }
}