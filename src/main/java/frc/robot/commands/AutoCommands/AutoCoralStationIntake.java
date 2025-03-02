package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CoralAlgaeCommands.IntakeCoral;
import frc.robot.subsystems.CoralAlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralAlgaeSubsystem.CoralPivotPositions;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;
import frc.robot.commands.AutoCommands.AutoScoringPosition;

public class AutoCoralStationIntake extends SequentialCommandGroup {

    public AutoCoralStationIntake(CoralAlgaeSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem) {

        addCommands(
                new ParallelDeadlineGroup(
                        new InstantCommand(() -> elevatorSubsystem.setPIDPosition(ElevatorPositions.CoralStation)),
                        new InstantCommand(() -> coralSubsystem.setPIDPosition(CoralPivotPositions.CoralStation))),
                new IntakeCoral(coralSubsystem, 1));
    }
}
