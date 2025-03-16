package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralAlgaeSubsystem.CoralPivotPositions;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;

public class ScoringPosition extends Command {

    CoralAlgaeSubsystem coralSubsystem;
    CoralPivotPositions coralPivotPositions;
    ElevatorSubsystem elevatorSubsystem;
    ElevatorPositions elevatorPosition;

    public ScoringPosition(CoralAlgaeSubsystem coralSubsystem, CoralPivotPositions coralPivotPositions,
            ElevatorSubsystem elevatorSubsystem, ElevatorPositions elevatorPosition) {
        this.coralSubsystem = coralSubsystem;
        this.coralPivotPositions = coralPivotPositions;
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorPosition = elevatorPosition;
        addRequirements(coralSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPIDPosition(elevatorPosition);
        // elevatorSubsystem.setProfiledPIDPosition(elevatorPosition);
        // elevatorSubsystem.setTargetPosition(elevatorPosition);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        // switch (elevatorSubsystem.getElevatorTarget()) {
        // case L4:
        // coralSubsystem.setPIDPosition(CoralPivotPositions.L4);
        // case L3:
        // coralSubsystem.setPIDPosition(CoralPivotPositions.L3);
        // case L2:
        // coralSubsystem.setPIDPosition(CoralPivotPositions.L2);
        // case L1:
        // coralSubsystem.setPIDPosition(CoralPivotPositions.L1);
        // default:
        // coralSubsystem.setPIDPosition(CoralPivotPositions.Stow);
        // }
        // try {
        // wait(1000);
        // } catch (InterruptedException e) {
        // // TODO Auto-generated catch block
        // e.printStackTrace();
        // }
        coralSubsystem.setPIDPosition(coralPivotPositions);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getElevatorPosition() > 0.4 * elevatorPosition.getValueRotations();
    }
}