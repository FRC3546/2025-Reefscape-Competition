package frc.robot.commands.ManualCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualCoralAlignment extends Command {

    SwerveSubsystem swerveSubsystem;
    double speed;

    public ManualCoralAlignment(SwerveSubsystem swerveSubsystem, double speed) {
        this.swerveSubsystem = swerveSubsystem;
        this.speed = speed;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        swerveSubsystem.drive(new Translation2d(0,speed), 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(0,0), 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}