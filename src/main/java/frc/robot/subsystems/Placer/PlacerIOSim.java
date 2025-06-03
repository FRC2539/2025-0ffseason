package frc.robot.subsystems.Placer;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.constants.PlacerConstants;

public class PlacerIOSim implements PlacerIO{
    
    private double voltage;
    private double speed;

    private LoggedNetworkBoolean placerGPSensorInitial =
            new LoggedNetworkBoolean("Gripper Initial Sensor Sim", true);

    private LoggedNetworkBoolean placerGPSensorSecond =
            new LoggedNetworkBoolean("Gripper Second Sensor Sim", true);


    public void updateInputs(PlacerIOInputs inputs) {
        inputs.voltage = voltage;
        inputs.speed = speed;
        inputs.firstSensor = placerGPSensorInitial.get(); 
        inputs.secondSensor = placerGPSensorSecond.get();
    }

    public void setVoltage(double v) {
        this.voltage = v;
    }
}
