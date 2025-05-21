package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOKraken {
    
    private TalonFX elevatorLeader = new TalonFX(0, "fillOut");
    private TalonFX elevatorFollower = new TalonFX(1, "fillOut"); //Set IDs later

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public ElevatorIOKraken(){

        elevatorFollower.setPosition(0);


    }

    public void setVoltage(double voltage) {
        elevatorLeader.setVoltage(voltage);
    }

    public void setPosition(double position) {
        if (position > 666) { // set to upper limit
            position = 666;
        }

        if (position < 0) { // set to lower limit
            position = 0;
        }
        
        elevatorLeader.setControl(motionMagicVoltage);
    }

}
