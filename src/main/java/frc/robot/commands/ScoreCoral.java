package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralAlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.CoralAlgaeCommands.OuttakeCoral;

public class ScoreCoral extends SequentialCommandGroup{

    public ScoreCoral(ElevatorSubsystem elevatorSubsystem, CoralAlgaeSubsystem coralSubsystem){
        addCommands(
            new OuttakeCoral(coralSubsystem, 1).withTimeout(2),
            new Stow(coralSubsystem, elevatorSubsystem)
        );
    }
    
}
