package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;


public class ElevatorIOTalonFX implements ElevatorIO{
    
    private TalonFX elevatorLeftMotor = new TalonFX(ElevatorConstants.elevatorLeftMotorId); // Leader
    private TalonFX elevatorRightMotor = new TalonFX(ElevatorConstants.elevatorRightMotorId); // Follower

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public ElevatorIOTalonFX(){
        elevatorLeftMotor.setPosition(0);
        elevatorRightMotor.setPosition(0);
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

        elevatorLeftMotor.getConfigurator().apply(config);
        elevatorRightMotor.getConfigurator().apply(config);


        elevatorRightMotor.setControl(new Follower(elevatorLeftMotor.getDeviceID(), true));

        elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(ElevatorIOInputs inputs) {

        inputs.position = elevatorLeftMotor.getPosition().refresh().getValueAsDouble();
        inputs.voltage = elevatorLeftMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.speed = elevatorLeftMotor.getVelocity().refresh().getValueAsDouble();
        inputs.temperature = elevatorLeftMotor.getDeviceTemp().getValueAsDouble();
        inputs.current = elevatorLeftMotor.getStatorCurrent().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        elevatorLeftMotor.setVoltage(voltage);
    }

    public void setPosition(double position) {
        if (position > ElevatorConstants.upperLimit) { // set to ElevatorConstants upper limit
            position = ElevatorConstants.upperLimit;
        }

        if (position < ElevatorConstants.lowerLimit) { // set to ElevatorConstants lower limit
            position = ElevatorConstants.lowerLimit;
        }
        
        elevatorLeftMotor.setControl(motionMagicVoltage);
    }

    public void resetPosition(){
        elevatorLeftMotor.setPosition(0);
    }

}
