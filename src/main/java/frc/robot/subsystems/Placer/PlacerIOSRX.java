package frc.robot.subsystems.Placer;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.constants.PlacerConstants;

public class PlacerIOSRX implements PlacerIO{

    private double voltage;

    // WPI_TalonSRX x =  (PlacerConstants.placerMotorID, PlacerConstants.placerMotorCanbus);

    @AutoLogOutput
    private DigitalInput pieceSeatedSensor = new DigitalInput(PlacerConstants.secondSensorChannel);
    @AutoLogOutput
    private DigitalInput hasPieceSensor = new DigitalInput(PlacerConstants.sensorChannel);

    public PlacerIOSRX(){
        

        Shuffleboard.getTab("debug")
                .addBoolean("first sensor value", () -> hasPieceSensor.get());
        
        // placerMotor.setNeutralMode(NeutralModeValue.Brake);
        // placerMotor.getConfigurator().apply(new TalonFXConfiguration()); //Default
    }

    public void updateInputs(PlacerIOInputs inputs){
        inputs.voltage = 0; //placerMotor.getMotorVoltage().getValueAsDouble(); 
        inputs.current = 0; //placerMotor.getSupplyCurrent().getValueAsDouble(); 
        inputs.temperature =  0; // placerMotor.getDeviceTemp().getValueAsDouble();

        //placerMotor.setVoltage(voltage);

        inputs.firstSensor = hasPieceSensor.get();
        inputs.secondSensor = pieceSeatedSensor.get();
    }

    @AutoLogOutput
    public boolean getSensorValue(){
        return hasPieceSensor.get();
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
