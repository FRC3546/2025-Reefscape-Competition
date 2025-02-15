package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem.CoralPivotPositions;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;

public class CoralStationIntake extends SequentialCommandGroup {

    public CoralStationIntake(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem) {

        addCommands(
                new ParallelDeadlineGroup(
                        new InstantCommand(() -> elevatorSubsystem.setPIDPosition(ElevatorPositions.CoralStation)),
                        new InstantCommand(() -> coralSubsystem.setPIDPosition(CoralPivotPositions.CoralStation))),
                new IntakeCoral(coralSubsystem, 0.7),
                new Stow(coralSubsystem, elevatorSubsystem));
    }

}
