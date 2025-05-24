package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase{
    
    private ElevatorIO pivotIO;
    private ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    private SysIdRoutine elevatorSysIdRoutine =
            new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(null, null, null));

    public ElevatorSubsystem(ElevatorIO elevatorIO){
        this.pivotIO = elevatorIO;
        setDefaultCommand(setVoltage(0));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return elevatorSysIdRoutine.quasistatic(direction);
        }

    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return elevatorSysIdRoutine.dynamic(direction);
    }

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
