package frc.robot.subsystems.elevator;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    
    private ElevatorIO pivotIO;
    private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();



    public ElevatorSubsystem(ElevatorIO elevatorIO){
        this.pivotIO = elevatorIO;
        setDefaultCommand(setVoltage(0));
    }

    @Override
    public void periodic() {

        pivotIO.updateInputs(elevatorInputs);
        Logger.processInputs("RealOutputs/Elevator", elevatorInputs);
    }

    public Command zeroElevatorCommand() {
        return runOnce(
                () -> {
                    pivotIO.resetPosition();
                });
    }

    public Command moveElevatorUp() {
        return setVoltage(12);
    }

    public Command moveElevatorDown() {
        return setVoltage(-12);
    }

    public Command setVoltage(double voltage) {
        return run(
                () -> {
                    pivotIO.setVoltage(voltage);
                });
    }

    public Command setPosition(double position) {
        return Commands.runOnce(
                () -> {
                    pivotIO.setPosition(position);
                }, this);
    }

    public double getPosition() {
        return elevatorInputs.position;
    }
}
