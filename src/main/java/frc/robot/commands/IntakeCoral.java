package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class IntakeCoral extends Command {

    CoralSubsystem coralSubsystem;
    double speed;

    public IntakeCoral(CoralSubsystem coralSubsystem, double speed) {
        this.coralSubsystem = coralSubsystem;
        this.speed = Math.abs(speed);
        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {
        coralSubsystem.setIntakeMotorSpeed(speed);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.stopIntakeMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}