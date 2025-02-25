package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralAlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.CoralAlgaeCommands.OuttakeAlgae;

public class ScoreAlgae extends SequentialCommandGroup{

    public ScoreAlgae(ElevatorSubsystem elevatorSubsystem, CoralAlgaeSubsystem coralAlgaeSubsystem){
        addCommands(
            new OuttakeAlgae(coralAlgaeSubsystem, 1).withTimeout(0.75),
            new Stow(coralAlgaeSubsystem, elevatorSubsystem)
        );
    }
    
}
