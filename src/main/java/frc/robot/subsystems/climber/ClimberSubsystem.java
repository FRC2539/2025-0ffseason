package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ClimberSubsystem extends SubsystemBase {

    private ClimberIO climberIO;
    private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    // NetworkTableInstance nInstance = NetworkTableInstance.getDefault();
    // NetworkTable table = nInstance.getTable("SmartDashboard");
    // NetworkTableValue climbervoltage = table.getValue("climbervoltage");

    LoggedNetworkNumber climbervoltage = new LoggedNetworkNumber("Climber Voltage", 0);
    LoggedNetworkNumber climberHeadVoltage =
            new LoggedNetworkNumber("Climber Head (Winch) Voltage", 0);

    public ClimberSubsystem(ClimberIO climberIO) {

        this.climberIO = climberIO;

        setDefaultCommand(stopClimber());
    }

    public void periodic() {

        climberIO.updateInputs(climberInputs);

        Logger.processInputs("RealOutputs/Climber", climberInputs);
        
        // if (climberInputs.voltage < 0 && climberInputs.position <= lowerLimit) {
        //     this.piviotIO.setVoltage(0);
        // }

        // if (climberInputs.voltage > 0 && climberInputs.position >= upperLimit) {
        //     this.piviotIO.setVoltage(0);
        // }
    }

    // Here begins the winch climber motor commands
    public Command zeroClimberCommand() {
        return runOnce(
                () -> {
                    climberIO.resetPosition(0);
                });
    }

    public Command moveClimberUpVoltage() {
        return setClimberVoltage(8);
    }

    public Command climberTuneable() {
        return run(
                () -> {
                    double voltage = climbervoltage.get();
                    climberIO.setVoltage(voltage);
                });
    }

    public Command moveClimberDownVoltage() {
        return setClimberVoltage(-8);
    }

    public Command setClimberVoltage(double voltage) {
        return run(
                () -> {
                    climberIO.setVoltage(voltage);
                });
    }

    public Command setPosition(double position) {
        return run(
                () -> {
                    climberIO.setPosition(position);
                });
    }

    public double getPosition() {
        return climberInputs.position;
    }

    public Command upPosition() {
        return run(
                () -> {
                    climberIO.setPosition(ClimberConstants.upperLimit);
                });
    }

    public Command downPosition() {
        return run(
                () -> {
                    climberIO.setPosition(ClimberConstants.lowerLimit);
                });
    }

    public Command stopClimber() {
        return run(
                () -> {
                    climberIO.setVoltage(0);
       
                });
    }

    // Here begins climber head commands
    
}