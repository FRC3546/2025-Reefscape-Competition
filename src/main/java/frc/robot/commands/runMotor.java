package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class runMotor extends Command {

    CoralSubsystem coralSubsystem;
    double speed;

    public runMotor(CoralSubsystem coralSubsystem, double speed){
        this.coralSubsystem = coralSubsystem;
        this.speed = speed;
        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {
        coralSubsystem.setMotorSpeed(speed);
    }
 
    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.stopPivotMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}