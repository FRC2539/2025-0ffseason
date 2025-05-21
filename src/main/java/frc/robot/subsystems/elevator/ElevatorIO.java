package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    public class ClimberIOInputs {
        public double position = 0;
        public double speed = 0;
        public double voltage;
        public double temperature = 0;
        public double current = 0;
    }
    

    public void setPosition(double pos);

    public void setSpeed(double speed);

    public void setVoltage(double vol);
}