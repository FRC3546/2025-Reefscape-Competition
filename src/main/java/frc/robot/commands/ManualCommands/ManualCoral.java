package frc.robot.commands.ManualCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAlgaeSubsystem;

public class ManualCoral extends Command {
    private CoralAlgaeSubsystem coralSubsystem;
    private DoubleSupplier speed;

    public ManualCoral(CoralAlgaeSubsystem coralSubsystem, DoubleSupplier speed) {
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
