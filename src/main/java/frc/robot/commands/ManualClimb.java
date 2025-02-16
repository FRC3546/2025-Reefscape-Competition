package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualClimb extends Command {
    private ClimberSubsystem climberSubsystem;
    private DoubleSupplier speed;

    public ManualClimb(ClimberSubsystem climberSubsystem, DoubleSupplier speed) {
        this.climberSubsystem = climberSubsystem;
        this.speed = speed;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climberSubsystem.setClimberSpeed(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopClimberMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
