package frc.robot.commands;

import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CANdleSubsystem.Color;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ledTestCommand extends Command {
  private final CANdleSubsystem candleSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ledTestCommand(CANdleSubsystem candleSubsystem) {
    this.candleSubsystem = candleSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(candleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    candleSubsystem.clear();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    candleSubsystem.strobe(Color.green);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    candleSubsystem.clear();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return false;
  }
}