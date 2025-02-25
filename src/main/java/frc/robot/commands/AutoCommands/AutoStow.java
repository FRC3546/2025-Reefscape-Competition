package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralAlgaeSubsystem.CoralPivotPositions;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;

public class AutoStow extends Command {

    CoralAlgaeSubsystem coralAlgaeSubsystem;
    ElevatorSubsystem elevatorSubsystem;

    public AutoStow(CoralAlgaeSubsystem coralAlgaeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.coralAlgaeSubsystem = coralAlgaeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(coralAlgaeSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPIDPosition(ElevatorPositions.Stow);
        coralAlgaeSubsystem.setPIDPosition(CoralPivotPositions.Stow);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}