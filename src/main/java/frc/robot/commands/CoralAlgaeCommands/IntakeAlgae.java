package frc.robot.commands.CoralAlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralAlgaeSubsystem;

public class IntakeAlgae extends Command {

    CoralAlgaeSubsystem coralSubsystem;
    double speed;

    public IntakeAlgae(CoralAlgaeSubsystem coralSubsystem, double speed) {
        this.coralSubsystem = coralSubsystem;
        this.speed = (Math.abs(speed));
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
        // return (coralSubsystem.getAlgaeCurrent() > 10) && (coralSubsystem.getAlgaeCurrent() < 20);
        return false;
    }
}