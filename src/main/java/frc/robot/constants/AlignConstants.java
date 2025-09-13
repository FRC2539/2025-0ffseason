package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class AlignConstants {
  public static final double Kp = 1;
  public static final double Ki = 0.0;
  public static final double Kd = 0.0;

  public static final double leftOffset = -0.2;
  public static final double rightOffset = 0.2;
  public static final double centerOffset = -0.3;
  public static final double reefDistance = Units.inchesToMeters(33.25) / 2;

  public static final double aligningAngleTolerance = Units.degreesToRadians(3);
  public static final double aligningXTolerance = Units.inchesToMeters(2);
  public static final double aligningYTolerance = Units.inchesToMeters(1);

  public static boolean robotInPlace(Pose2d robotPose, Pose2d targetPose, double offset) {
    Pose2d alignmentPose =
        targetPose.plus(
            new Transform2d(
                new Translation2d(AlignConstants.reefDistance, offset), Rotation2d.k180deg));
    Pose2d offsetPose = robotPose.relativeTo(alignmentPose);
    return (offsetPose.getX() > -aligningXTolerance)
        && (Math.abs(offsetPose.getY()) < aligningYTolerance)
        && ((Math.abs(offsetPose.getRotation().getRadians()) % (2 * Math.PI))
            < aligningAngleTolerance);
  }
}