package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;

public class AutoReefAlignLeft extends Command {

    SwerveSubsystem swerveSubsystem;

    public AutoReefAlignLeft(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.autoAlign(Constants.reefScoreLocation.LEFT);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new ChassisSpeeds(0,0,0));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}