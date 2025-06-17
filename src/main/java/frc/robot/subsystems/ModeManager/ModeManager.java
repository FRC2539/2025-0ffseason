package frc.robot.subsystems.modeManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.placer.PlacerSubsystem;

public class ModeManager extends SubsystemBase {

  private ElevatorSubsystem elevator;

  public ModeManager(ElevatorSubsystem elevator, PlacerSubsystem placer) {
    this.elevator = elevator;
  }

  public static enum Position {
    L1(0),
    L2(0),
    L3(0),
    L4(7.423),

    Algae2(0),
    Algae3(0),

    Home(0),
    Start(0);

    private double elevatorHeight;
        
    private Position(double elevatorHeight) {
      this.elevatorHeight = elevatorHeight;
    }

    public double elevatorHeight() {
      return elevatorHeight;
    }
  }

  public Command moveElevator(Position position) {
    return Commands.runOnce(
      () -> elevator.setPosition(position.elevatorHeight), this.elevator
    );
  }
}