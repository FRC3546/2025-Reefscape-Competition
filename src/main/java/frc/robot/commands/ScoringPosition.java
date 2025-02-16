package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem.CoralPivotPositions;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;

public class ScoringPosition extends Command {

    CoralSubsystem coralSubsystem;
    CoralPivotPositions coralPivotPositions;
    ElevatorSubsystem elevatorSubsystem;
    ElevatorPositions elevatorPosition;

    public ScoringPosition(CoralSubsystem coralSubsystem, CoralPivotPositions coralPivotPositions, ElevatorSubsystem elevatorSubsystem, ElevatorPositions elevatorPosition) {
        this.coralSubsystem = coralSubsystem;
        this.coralPivotPositions = coralPivotPositions;
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorPosition = elevatorPosition;
        addRequirements(coralSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPIDPosition(elevatorPosition);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        // switch (elevatorSubsystem.getElevatorTarget()) {
        //     case L4:
        //         coralSubsystem.setPIDPosition(CoralPivotPositions.L4);
        //     case L3:
        //         coralSubsystem.setPIDPosition(CoralPivotPositions.L3);
        //     case L2:
        //         coralSubsystem.setPIDPosition(CoralPivotPositions.L2);
        //     case L1:
        //         coralSubsystem.setPIDPosition(CoralPivotPositions.L1);
        //     default:
        //         coralSubsystem.setPIDPosition(CoralPivotPositions.Stow);
        // }
        coralSubsystem.setPIDPosition(coralPivotPositions);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.encoderToInch() > 0.9*elevatorPosition.getValue();
    }
}