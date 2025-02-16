package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem.CoralPivotPositions;

public class ResetElevator extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private CoralSubsystem coralSubsystem;

    public ResetElevator(ElevatorSubsystem elevatorSubsystem,CoralSubsystem coralSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorSpeed(-0.8);
        coralSubsystem.setPIDPosition(CoralPivotPositions.Stow);
    }

    @Override
    public void execute() {
        if(elevatorSubsystem.getBackElevatorLimitSwitch()){
            elevatorSubsystem.setElevatorSpeed(-0.4);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setElevatorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getFrontElevatorLimitSwitch();
    }

}
