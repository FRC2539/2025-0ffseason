package frc.robot.subsystems.placer;

import org.littletonrobotics.junction.AutoLog;

public interface PlacerIO {

    @AutoLog
    public class PlacerIOInputs {
        public double voltage = 0;

        public boolean firstSensor = false;
        public boolean secondSensor = false;
    }

    public void setVoltage(double voltage);

    public void updateInputs(PlacerIOInputs placerInputs);

}
