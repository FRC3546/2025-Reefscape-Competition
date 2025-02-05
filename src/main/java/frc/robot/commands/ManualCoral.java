package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class ManualCoral extends Command {
    private CoralSubsystem coralSubsystem;
    private DoubleSupplier speed;

    public ManualCoral(CoralSubsystem coralSubsystem, DoubleSupplier speed) {
        this.coralSubsystem = coralSubsystem;
        this.speed = speed;
        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        coralSubsystem.setPivotMotorSpeed(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
