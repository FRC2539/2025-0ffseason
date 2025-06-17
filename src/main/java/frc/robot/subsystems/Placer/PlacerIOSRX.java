package frc.robot.subsystems.placer;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.type.PlaceholderForType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.constants.PlacerConstants;

public class PlacerIOSRX implements PlacerIO{

    private double voltage;

    TalonSRX placerMotor = new TalonSRX(PlacerConstants.placerMotorID);

    private DigitalInput pieceSeatedSensor = new DigitalInput(PlacerConstants.secondSensorChannel);
    private DigitalInput hasPieceSensor = new DigitalInput(PlacerConstants.sensorChannel);

    public PlacerIOSRX() {
        

        Shuffleboard.getTab("debug")
                .addBoolean("first sensor value", () -> hasPieceSensor.get());
        
        placerMotor.configAllSettings(new TalonSRXConfiguration());
        placerMotor.setNeutralMode(NeutralMode.Brake);

    }

    public void updateInputs(PlacerIOInputs inputs) {
        inputs.voltage = placerMotor.getMotorOutputVoltage();


        placerMotor.set(TalonSRXControlMode.PercentOutput, voltage / 12);

        inputs.firstSensor = !hasPieceSensor.get();
        inputs.secondSensor = !pieceSeatedSensor.get();
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
