package frc.robot.subsystems.modeManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
//import frc.robot.subsytems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.placer.PlacerSubsystem;

public class ModeManager extends SubsystemBase {

  private ElevatorSubsystem elevator;
  private PlacerSubsystem placer;

  public ModeManager(ElevatorSubsystem elevator, PlacerSubsystem placer) {
    this.elevator = elevator;
    this.placer = placer;
  }

  public static enum Position {
    L1(4),
    L2(11.25),
    L3(16.876), //13.42 // 15.049 measured
    L4(22.5),

    Home(1.6);
  

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