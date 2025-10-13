package frc.robot.subsystems.vision;

import frc.robot.commands.LimelightHelpers;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;

public class SuperVision {
    Distance cameraDistance = edu.wpi.first.units.Units.Inches.of(24);
    Distance cameraForwardOffset = edu.wpi.first.units.Units.Inches.of(13.5);
    double leftCameraAngle = 40.0;
    double rightCameraAngle = -40.0;

    double leftAngle = Math.toRadians(90 - leftCameraAngle - LimelightHelpers.getTX("limelight-left"));
    double rightAngle = Math.toRadians(90 - rightCameraAngle - LimelightHelpers.getTX("limelight-right"));

    public Distance frontBackDistance = edu.wpi.first.units.Units.Inches.of(cameraDistance.in(edu.wpi.first.units.Units.Inches) * Math.sin(leftAngle) * Math.sin(rightAngle) / Math.sin(rightAngle - leftAngle)).plus(cameraForwardOffset);
    public Distance leftRightDistance = edu.wpi.first.units.Units.Inches.of(cameraDistance.in(edu.wpi.first.units.Units.Inches) * Math.sin(leftAngle) * Math.cos(rightAngle) / Math.sin(rightAngle - leftAngle)).plus(cameraDistance.times(0.5));
}