package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem.CoralPivotPositions;

public class ScoreCoral extends Command {

    CoralSubsystem coralSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    CoralPivotPositions targetPivotPosition;
    double speed;

    public ScoreCoral(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, double speed) {
        targetPivotPosition = CoralPivotPositions.Stow;
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = -(Math.abs(speed));
        addRequirements(coralSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        switch (elevatorSubsystem.getElevatorTarget()) {
            case L4:
                targetPivotPosition = CoralPivotPositions.L4;
                coralSubsystem.setPIDPosition(CoralPivotPositions.L4);
                break;
            case L3:
                targetPivotPosition = CoralPivotPositions.L3;
                coralSubsystem.setPIDPosition(CoralPivotPositions.L3);
                break;
            case L2:
                targetPivotPosition = CoralPivotPositions.L2;
                coralSubsystem.setPIDPosition(CoralPivotPositions.L2);
            case L1:
                targetPivotPosition = CoralPivotPositions.L1;
                coralSubsystem.setPIDPosition(CoralPivotPositions.L1);
                break;

            default:
                targetPivotPosition = CoralPivotPositions.Stow;
                coralSubsystem.setPIDPosition(CoralPivotPositions.Stow);
                break;
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIntakeMotorSpeed(speed);
        Timer.delay(2);
        coralSubsystem.stopIntakeMotor();
    }

    @Override
    public boolean isFinished() {
        return coralSubsystem.pidWithinBounds(targetPivotPosition, 0.05);
    }
}