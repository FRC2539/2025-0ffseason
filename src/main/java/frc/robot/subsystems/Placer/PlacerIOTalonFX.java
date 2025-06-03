package frc.robot.subsystems.Placer;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.constants.PlacerConstants;

public class PlacerIOTalonFX implements PlacerIO{

    private double voltage;

    TalonFX placerLead = new TalonFX(PlacerConstants.placerMotorID, PlacerConstants.placerMotorCanbus);

    private AnalogInput pieceSensor = new AnalogInput(PlacerConstants.sensorChannel);
    private DigitalInput hasPieceSensor = new DigitalInput(PlacerConstants.secondSensorChannel);

    public PlacerIOTalonFX(){
        

        Shuffleboard.getTab("debug")
                .addInteger("first sensor value", () -> pieceSensor.getValue());
        
        placerLead.setNeutralMode(NeutralModeValue.Brake);
        placerLead.getConfigurator().apply(new TalonFXConfiguration()); //Default
    }

    public void updateInputs(PlacerIOInputs inputs){
        inputs.voltage = placerLead.getMotorVoltage().getValueAsDouble(); 
        inputs.current = placerLead.getSupplyCurrent().getValueAsDouble(); 
        inputs.temperature = placerLead.getDeviceTemp().getValueAsDouble();

        placerLead.setVoltage(voltage);

        inputs.firstSensor = pieceSensor.getValue() < 2800; //Probably needs to be corrected
        inputs.secondSensor = !hasPieceSensor.get();
    }

    @AutoLogOutput
    public int getSensorValue(){
        return pieceSensor.getValue();
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
