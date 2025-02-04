package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem.CoralPivotPositions;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;

public class Stow extends Command {

    CoralSubsystem coralSubsystem;
    ElevatorSubsystem elevatorSubsystem;

    public Stow(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(coralSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.pidSetPosition(ElevatorPositions.Stow);
        coralSubsystem.setPIDPosition(CoralPivotPositions.Stow);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}