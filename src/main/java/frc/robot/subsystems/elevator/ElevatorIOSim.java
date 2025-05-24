package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOSim implements ElevatorIO{

    private double position = 0;
    private double voltage = 0;
    private double temperature = 0;
    private double current = 0;
    private double speed = 0;
    private double positionSetpoint = 0;

    private PIDController pidController = new PIDController(5, 0, 0);

    private boolean positionControl = false;

    public void updateInputs(ElevatorIOInputs inputs) {
        if (positionControl) {
            voltage = pidController.calculate(position, positionSetpoint);
        }
    }

    public void setPosition(double pos){
        positionControl = true;
        position = pos;
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    public void setVoltage(double vol){
        positionControl = false;
        voltage = vol;
    }

    public void resetPosition() {
        position = 0;
    }
}
