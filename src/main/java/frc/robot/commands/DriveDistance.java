// DriveDistance.java
package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class DriveDistance extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final double targetDistanceMeters;
    private final double directionDegrees; // New: Field-centric direction in degrees
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    // PID for field-centric X movement
    private final PIDController xController = new PIDController(10.0, 0.0, 0.0); // Tune this P!
    // PID for field-centric Y movement
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0); // Tune this P!
    // PID for maintaining heading
    private final PIDController thetaController = new PIDController(2.0, 0.0, 0.0); // Tune this P!

    private Pose2d initialPose;
    private double initialHeadingRadians; // Robot's heading when this command starts

    /**
     * Drives the robot a specified distance in a given field-centric direction,
     * while maintaining its initial heading.
     * Requires an accurate field-centric pose estimate from the drivetrain.
     *
     * @param drivetrain The drivetrain subsystem.
     * @param distanceMeters The distance to drive in meters (positive).
     * @param directionDegrees The field-centric direction to drive in degrees (0 = +X, 90 = +Y, etc.).
     */
    public DriveDistance(CommandSwerveDrivetrain drivetrain, double distanceMeters, double directionDegrees) {
        this.drivetrain = drivetrain;
        this.targetDistanceMeters = distanceMeters;
        this.directionDegrees = directionDegrees;
        addRequirements(drivetrain);

        thetaController.enableContinuousInput(-Math.PI, Math.PI); // For angle wrapping

        xController.setTolerance(0.05); // meters
        yController.setTolerance(0.05); // meters
        thetaController.setTolerance(Units.degreesToRadians(2.0)); // degrees
    }

    // You can also provide a simpler constructor if you want to default to "forward" (0 degrees field-centric)
    // or relative to robot's initial heading, but the current design makes more sense for field-centric direction.
    // If you want "forward relative to robot", you'd call this constructor like:
    // new DriveDistance(drivetrain, distance, drivetrain.getPose().getRotation().getDegrees());

    @Override
    public void initialize() {
        initialPose = drivetrain.getRobotPose();
        initialHeadingRadians = initialPose.getRotation().getRadians();

        // Calculate the target robot-centric X and Y coordinates
        // The 'directionDegrees' is now relative to the robot's current heading
        double directionRadians = Units.degreesToRadians(directionDegrees);
        double targetRobotX = targetDistanceMeters * Math.cos(directionRadians);
        double targetRobotY = targetDistanceMeters * Math.sin(directionRadians);

        // Set PID setpoints for robot-centric target position and initial heading
        xController.setSetpoint(targetRobotX);
        yController.setSetpoint(targetRobotY);
        thetaController.setSetpoint(initialHeadingRadians);

        // Reset PIDs to clear any previous state
        xController.reset();
        yController.reset();
        thetaController.reset();

        //System.out.println("DriveDistance: Initializing. Target Field (X,Y)=(" +
                        //    String.format("%.2f", targetRobotX) + ", " + String.format("%.2f", targetRobotY) +
                        //    "), Target Heading=" + String.format("%.2f", Units.radiansToDegrees(initialHeadingRadians)) + "deg");
    }

    @Override
    public void execute() {

        Pose2d currentPose = drivetrain.getRobotPose();

        // Assuming `this.initialPose` holds the robot's pose when the command started.
        Translation2d deltaTranslation = currentPose.getTranslation().minus(this.initialPose.getTranslation());
        // Rotate this delta back to the initial robot's frame of reference
        Rotation2d initialRotationInverse = this.initialPose.getRotation().unaryMinus();
        Translation2d robotRelativeDelta = deltaTranslation.rotateBy(initialRotationInverse);

        double currentRobotXDisplacement = robotRelativeDelta.getX();
        double currentRobotYDisplacement = robotRelativeDelta.getY();

        // Now, calculate the velocities based on the error to the desired robot-centric displacement
        double xVelocity = xController.calculate(currentRobotXDisplacement); // PID tries to reach targetRobotX
        double yVelocity = yController.calculate(currentRobotYDisplacement); // PID tries to reach targetRobotY
        double thetaVelocity = thetaController.calculate(currentPose.getRotation().getRadians()); // PID still aims for initial absolute heading

        // Limit velocities to a safe maximum (tune these values!)
        double maxLinearSpeed = 0.5; // meters per second
        xVelocity = Math.max(-maxLinearSpeed, Math.min(xVelocity, maxLinearSpeed));
        yVelocity = Math.max(-maxLinearSpeed, Math.min(yVelocity, maxLinearSpeed));
        // You might also want to limit rotational velocity

        // Create robot-relative ChassisSpeeds directly
        // Since xVelocity and yVelocity are already robot-centric (forward/backward and strafe),
        // and thetaVelocity is the desired rotational speed, we can directly form ChassisSpeeds.
        ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity);

        // Apply these robot-relative speeds to the drivetrain
        drivetrain.setControl(applyRobotSpeeds.withSpeeds(robotRelativeSpeeds));

        // System.out.println("DriveDistance: Current Robot-Centric (X,Y)=(" +
        //                     String.format("%.2f", currentRobotXDisplacement) + ", " +
        //                     String.format("%.2f", currentRobotYDisplacement) +
        //                     "), Vx=" + String.format("%.2f", xVelocity) +
        //                     ", Vy=" + String.format("%.2f", yVelocity) +
        //                     ", Theta=" + String.format("%.2f", Units.radiansToDegrees(currentPose.getRotation().getRadians())) + " deg");
    }

    @Override
    public boolean isFinished() {
        // Command finishes when robot reaches target X, Y, and maintains heading
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.setControl(applyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
        // System.out.println("DriveDistance: Finished or Interrupted. Final Position: " + drivetrain.getRobotPose().getTranslation().toString());
    }
}