package frc.robot.subsystems.Placer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.PlacerConstants;

public class PlacerSubsystem extends SubsystemBase{

    private PlacerIO placerIO;

    private PlacerIOInputsAutoLogged placerInputs = new PlacerIOInputsAutoLogged();

    public final Trigger HAS_PIECE = new Trigger(this::hasPiece);
    public final Trigger PIECE_SEATED = new Trigger(this::isPieceSeated);

    public boolean DOUBLESENSORMODE = true;

    LoggedNetworkNumber placerVoltage =
        new LoggedNetworkNumber("Placer Motor Voltage", 0);

    public PlacerSubsystem(PlacerIO placerIO){
        this.placerIO = placerIO;
        setDefaultCommand(setVoltage(0));
    }

    @AutoLogOutput
    public boolean usingDoubleSensorMode(){
        return DOUBLESENSORMODE;
    }

    public Command ejectReverse(double voltage){
        return setVoltage(-voltage);
    }


    public void periodic(){
        placerIO.updateInputs(placerInputs);
        Logger.processInputs("RealOutputs/Placer", placerInputs);
    }

    public Command placerTuneable(){
        return run(
            () -> {
                double voltage = placerVoltage.get();
                placerIO.setVoltage(voltage);
            });
    }

    public Command intake(double voltage){
        return setVoltage(voltage);
    }

    public Command intakeUntilPieceSet(){
        return setVoltage(PlacerConstants.handoffVoltage)
        .until(() -> placerInputs.firstSensor)
        .andThen(
            setVoltage(PlacerConstants.slowHandoffVoltage)
                .until(() -> placerInputs.secondSensor));
    }

    public Command intakeUntilPieceOneSensorMode(){
        return setVoltage(1).until(() -> placerInputs.secondSensor);
    }

    public Command intakeUntilPieceContained(){
        return Commands.either(
            intakeUntilPieceSet(), intakeUntilPieceOneSensorMode(), () -> this.DOUBLESENSORMODE ); 
    }

    public Command placePiece(){
        return setVoltage(PlacerConstants.placeVoltage)
        .until((HAS_PIECE.negate()).and(PIECE_SEATED.negate()))
        .andThen(Commands.waitSeconds(0.3));
    }

    public Command setVoltage(double voltage){
        return Commands.run(() -> {
            placerIO.setVoltage(voltage);
        }, this);
    }

    public boolean isPieceSeated(){
        return placerInputs.firstSensor;
    }

    public boolean hasPiece(){
        return placerInputs.secondSensor;
    }

    public boolean intaking(){
        return placerInputs.voltage > 3;
    }
    
}