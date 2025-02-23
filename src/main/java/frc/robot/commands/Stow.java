package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralAlgaeSubsystem.CoralPivotPositions;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;

public class Stow extends Command {

    CoralAlgaeSubsystem coralSubsystem;
    ElevatorSubsystem elevatorSubsystem;

    public Stow(CoralAlgaeSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(coralSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPIDPosition(ElevatorPositions.Stow);
        coralSubsystem.setPIDPosition(CoralPivotPositions.Stow);
    }

    @Override
    public void execute() {
        if(!coralSubsystem.getCoralSensor()){
            coralSubsystem.setIntakeMotorSpeed(-0.3);
        }
        else{
            coralSubsystem.stopIntakeMotor();
        }
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