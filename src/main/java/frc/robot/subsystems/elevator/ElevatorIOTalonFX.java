package frc.robot.subsystems.elevator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;


public class ElevatorIOTalonFX implements ElevatorIO{
    
    private TalonFX elevatorLeftMotor = new TalonFX(ElevatorConstants.elevatorLeftMotorId); // Leader
    private TalonFX elevatorRightMotor = new TalonFX(ElevatorConstants.elevatorRightMotorId); // Follower

    double target = 0;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    public ElevatorIOTalonFX() {
        elevatorLeftMotor.setPosition(0);
        elevatorRightMotor.setPosition(0);
        
        elevatorRightMotor.setControl(new Follower(elevatorLeftMotor.getDeviceID(), true));
        TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();
        //leftMotorConfigs.MotorOutput.OpenLoopRamp = 0.2;
        rightMotorConfigs.MotorOutput.PeakForwardDutyCycle = 1.0;
        rightMotorConfigs.MotorOutput.PeakReverseDutyCycle = -1.0;

        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 3; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        


        
        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        talonFXConfigs.MotionMagic.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        elevatorLeftMotor.getConfigurator().apply(talonFXConfigs);
        elevatorRightMotor.getConfigurator().apply(talonFXConfigs);


        elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorRightMotor.getConfigurator().apply(rightMotorConfigs);
        
        
    }

    public void updateInputs(ElevatorIOInputs inputs) {

        inputs.position = elevatorLeftMotor.getPosition().refresh().getValueAsDouble();
        inputs.voltage = elevatorLeftMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.speed = elevatorLeftMotor.getVelocity().refresh().getValueAsDouble();
        inputs.temperature = elevatorLeftMotor.getDeviceTemp().getValueAsDouble();
        inputs.current = elevatorLeftMotor.getStatorCurrent().getValueAsDouble();

        MotionMagicVoltage goal = m_request.withPosition(target).withSlot(0);
        elevatorLeftMotor.setControl(goal);
        //elevatorRightMotor.setControl(goal);
        
        //System.out.println("target: "+target);
        //System.out.println("left: "+elevatorLeftMotor.getPosition().refresh().getValueAsDouble());
        //System.out.println("right: "+elevatorRightMotor.getPosition().refresh().getValueAsDouble());
    }

    public void setVoltage(double voltage) {
        //elevatorLeftMotor.setVoltage(voltage);
        //elevatorRightMotor.setVoltage(-voltage);
    }

    public void setPosition(double position) {
        // if (position > ElevatorConstants.upperLimit) { // set to ElevatorConstants upper limit
        //     position = ElevatorConstants.upperLimit;
        // }

        // if (position < ElevatorConstants.lowerLimit) { // set to ElevatorConstants lower limit
        //     position = ElevatorConstants.lowerLimit;
        // }
        
        this.target = position;
    }

    public void resetPosition(){
        elevatorLeftMotor.setPosition(0);
    }

}
