package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralAlgaeSubsystem.CoralPivotPositions;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;

public class BargeScore extends Command {

    CoralAlgaeSubsystem coralSubsystem;
    CoralPivotPositions coralPivotPositions;
    ElevatorSubsystem elevatorSubsystem;
    ElevatorPositions elevatorPosition;

    public BargeScore(CoralAlgaeSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.coralSubsystem = coralSubsystem;
        coralPivotPositions = CoralPivotPositions.Barge;
        this.elevatorSubsystem = elevatorSubsystem;
        elevatorPosition = ElevatorPositions.Barge;
        addRequirements(coralSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPIDPosition(elevatorPosition);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setPIDPosition(coralPivotPositions);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.encoderToInch() > 85;
    }
}