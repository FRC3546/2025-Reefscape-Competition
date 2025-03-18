package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralAlgaeSubsystem.CoralPivotPositions;

public class ResetElevator extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private CoralAlgaeSubsystem coralSubsystem;

    public ResetElevator(ElevatorSubsystem elevatorSubsystem, CoralAlgaeSubsystem coralSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorSpeed(-0.3);
        coralSubsystem.setPIDPosition(CoralPivotPositions.Stow);
    }

    @Override
    public void execute() {
        if (elevatorSubsystem.getFrontElevatorLimitSwitch()) {
            elevatorSubsystem.setElevatorSpeed(-0.3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setElevatorSpeed(0);
        elevatorSubsystem.zeroElevatorPosition();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getFrontElevatorLimitSwitch();
    }

}
