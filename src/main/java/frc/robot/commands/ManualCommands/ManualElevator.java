package frc.robot.commands.ManualCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevator extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private DoubleSupplier speed;

    public ManualElevator(ElevatorSubsystem elevatorSubsystem, DoubleSupplier speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if (elevatorSubsystem.getFrontElevatorLimitSwitch() && -speed.getAsDouble() < 0) {
            elevatorSubsystem.setElevatorSpeed(0);
        }

        else {
            elevatorSubsystem.setElevatorSpeed(-speed.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
