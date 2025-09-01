package frc.robot.subsystems.placer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.PlacerConstants;

public class PlacerSubsystem extends SubsystemBase {

    private PlacerIO placerIO;

    private PlacerIOInputsAutoLogged placerInputs = new PlacerIOInputsAutoLogged();

    public final Trigger hasPiece = new Trigger(this::hasPiece);
    public final Trigger isPieceSeated = new Trigger(this::isPieceSeated);

    public PlacerSubsystem(PlacerIO placerIO) {
        this.placerIO = placerIO;
        setDefaultCommand(setVoltage(0));
    }

    public Command ejectReverse(double voltage) {
        return setVoltage(-voltage);
    }
    
    @Override
    public void periodic() {
        placerIO.updateInputs(placerInputs);
        Logger.processInputs("RealOutputs/Placer", placerInputs);

    }

    public Command intake(double voltage) {
        return setVoltage(voltage);
    }

    public Command intakeUntilPieceSet() {
        // return setVoltage(PlacerConstants.handoffVoltage)
        //     .until(() -> hasPiece.getAsBoolean())
        //     .andThen(Commands.sequence(Commands.waitSeconds(0.00), setVoltage(PlacerConstants.slowHandoffVoltage)
        //     .until(() -> isPieceSeated.getAsBoolean())));

        Command one = setVoltage(PlacerConstants.handoffVoltage).until(hasPiece);
        Command two = setVoltage(PlacerConstants.slowHandoffVoltage).until(isPieceSeated);

        return one.andThen(two).andThen(setVoltage(0));
    }

    public Command placePiece() {
        return setVoltage(PlacerConstants.placeVoltage)
        .until((hasPiece.negate()).and(isPieceSeated.negate()))
        .andThen(Commands.waitSeconds(0.3));
    }

    public Command setVoltage(double voltage) {
        return Commands.run(() -> {
            placerIO.setVoltage(voltage);
        }, this);
    }

    @AutoLogOutput
    public boolean isPieceSeated() {
        return placerInputs.secondSensor;
    }

    @AutoLogOutput
    public boolean hasPiece() {
        return placerInputs.firstSensor; 
    }

    public boolean intaking() {
        return placerInputs.voltage > 3;
    }
    
}