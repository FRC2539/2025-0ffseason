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
    L1(3.79),
    L2(7.15),
    L3(13.42),
    L4(22.18),

    Home(0);
  

    private double elevatorHeight;
        
    private Position(double elevatorHeight) {
      this.elevatorHeight = elevatorHeight;
    }

    public double elevatorHeight() {
      return elevatorHeight;
    }
  }

  public Command moveElevator(Position position) {
    return elevator.setPosition(position.elevatorHeight());
  }
}