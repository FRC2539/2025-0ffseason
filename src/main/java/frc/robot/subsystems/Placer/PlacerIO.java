package frc.robot.subsystems.Placer;

import org.littletonrobotics.junction.AutoLog;

public interface PlacerIO {

    @AutoLog
    public class PlacerIOInputs {
        public double speed = 0;
        public double voltage = 0;
        public double temperature = 0;
        public double current = 0;

        public boolean firstSensor = false;
        public boolean secondSensor = false;
    }

    public void setVoltage(double voltage);

    public void updateInputs(PlacerIOInputs placerInputs);

}
