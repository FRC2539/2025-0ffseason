package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class GlobalConstants {

    private static final SwerveModuleConstants EXAMPLE_MODULE = TunerConstants.FrontLeft;

    public static final LinearVelocity MAX_TRANSLATIONAL_SPEED = TunerConstants.kSpeedAt12Volts;
    //public static final AngularVelocity MAX_ROTATIONAL_SPEED = RotationsPerSecond.of(1);

    public static final Mass ROBOT_MASS = Pounds.of(150);
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(11.61);

    public static final double g = 9.81;
    public static final double STOPPING_TIME = 2;

    // negative for closer to the back of the robot, 0 for the center, positive for closer to the
    // front.
    // Should be <= bumperWidth/2 and >= -bumperWidth/2
    public static final double robotComXOffset = 0.20;

    // negative for closer to the right of the robot, 0 for the center, positive for closer to the
    // left.
    // Should be <= bumperLength/2 and >= -bumperLength/2
    public static final double robotComYOffset = -0.24;

    public static final double bumperLength = 0.97;
    public static final double bumperWidth = 0.91;

    public static final double COEFFICIENT_OF_FRICTION = 1.0;

    public static final DCMotor DRIVE_MOTOR =
            DCMotor.getKrakenX60Foc(1).withReduction(EXAMPLE_MODULE.DriveMotorGearRatio);
    public static final DCMotor STEER_MOTOR =
            DCMotor.getKrakenX60Foc(1).withReduction(EXAMPLE_MODULE.SteerMotorGearRatio);

    public static class ControllerConstants {
        public static final int LEFT_DRIVE_CONTROLLER = 0;
        public static final int RIGHT_DRIVE_CONTROLLER = 1;
        public static final int OPERATOR_CONTROLLER = 2;
    }

    public static final int[] blueReefTagIDs = {17, 18, 19, 20, 21, 22};
    public static final int[] redReefTagIDs = {6, 7, 8, 9, 10, 11};


    // public static RobotConfig getRobotConfigPathplanner() {
    //     if (robotConfigPathplanner == null) {
    //         try{
    //           robotConfigPathplanner = RobotConfig.fromGUISettings();
    //         } catch (Exception e) {
    //           // Handle exception as needed
    //           e.printStackTrace();
    //           throw new RuntimeException("Failed to load robot config from pathplanner.");
    //         }
    //     }
    //     return robotConfigPathplanner;
    // }
}