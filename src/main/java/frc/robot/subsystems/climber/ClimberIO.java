package frc.robot.subsystems.climber;



public interface ClimberIO {

    public void updateInputs(ClimberIOInputs inputs);


    public class ClimberIOInputs {
        public double position = 0;
        public double speed = 0;
        public double voltage;
        public double temperature = 0;
        public double current = 0;
    }

    // public void setPosition(double position);

    public void setVoltage(double voltage);

    public void resetPosition(double position);
}
