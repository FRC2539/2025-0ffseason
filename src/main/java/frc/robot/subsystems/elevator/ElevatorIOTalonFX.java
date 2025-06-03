package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorIOTalonFX implements ElevatorIO{
    
    //Set IDs later
    private TalonFX elevatorLeader = new TalonFX(0, ElevatorConstants.elevatorLeaderCanbus);
    private TalonFX elevatorFollower = new TalonFX(1, ElevatorConstants.elevatorFollowerCanbus); 

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public ElevatorIOTalonFX(){
        elevatorFollower.setPosition(0);

        motionMagicVoltage.withSlot(0);

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
                new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ElevatorConstants.upperLimit)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(ElevatorConstants.lowerLimit);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(0.56250);

        TalonFXConfiguration config =
                new TalonFXConfiguration()
                        .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                        .withSlot0(ElevatorConstants.slot0Configs)
                        .withMotionMagic(ElevatorConstants.motionMagicConfigs)
                        .withCurrentLimits(ElevatorConstants.currentLimit)
                        .withFeedback(feedbackConfigs);

        elevatorLeader.getConfigurator().apply(config);
        elevatorFollower.getConfigurator().apply(config);

        elevatorFollower.setControl(new Follower(elevatorFollower.getDeviceID(), false));

        elevatorLeader.setNeutralMode(NeutralModeValue.Brake);
        elevatorFollower.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(ElevatorIOInputs inputs) {

        inputs.position = elevatorLeader.getPosition().refresh().getValueAsDouble();
        inputs.voltage = elevatorLeader.getMotorVoltage().refresh().getValueAsDouble();
        inputs.speed = elevatorLeader.getVelocity().refresh().getValueAsDouble();
        inputs.temperature = elevatorLeader.getDeviceTemp().getValueAsDouble();
        inputs.current = elevatorLeader.getStatorCurrent().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        elevatorLeader.setVoltage(voltage);
    }

    public void setPosition(double position) {
        if (position > ElevatorConstants.upperLimit) { // set to ElevatorConstants upper limit
            position = ElevatorConstants.upperLimit;
        }

        if (position < ElevatorConstants.lowerLimit) { // set to ElevatorConstants lower limit
            position = ElevatorConstants.lowerLimit;
        }
        
        elevatorLeader.setControl(motionMagicVoltage);
    }

    public void resetPosition(){
        elevatorLeader.setPosition(0);
    }

}
