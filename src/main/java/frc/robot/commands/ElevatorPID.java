package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorPID extends Command {

    ElevatorSubsystem elevatorSubsystem;
    ElevatorPositions elevatorPosition;
    // ProfiledPIDController pidController = new ProfiledPIDController(5, 0, 0,
    //         new TrapezoidProfile.Constraints(0.5, 0.25));

    public ElevatorPID(ElevatorSubsystem elevatorSubsystem, ElevatorPositions elevatorPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorPosition = elevatorPosition;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setProfiledPIDPosition(elevatorPosition);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setElevatorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}