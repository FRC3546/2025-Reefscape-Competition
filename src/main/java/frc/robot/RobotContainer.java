// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.IntakeCoral;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.CoralPivotPositions;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.OuttakeCoral;
import frc.robot.commands.runMotor;

public class RobotContainer {
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private CommandJoystick coralJoystick = new CommandJoystick(0);
  public RobotContainer() {
    configureBindings();
  }

  public void updateDashboard(){
    SmartDashboard.putNumber("Coral Pivot Encoder", coralSubsystem.getPivotPosition());
    SmartDashboard.putNumber("PID Target", 0);
    SmartDashboard.putNumber("enum value", CoralPivotPositions.L1.getValue());
  }

  private void configureBindings() {
    coralJoystick.button(7).toggleOnTrue((new IntakeCoral(coralSubsystem, 0.3)));
    coralJoystick.button(9).toggleOnTrue((new OuttakeCoral(coralSubsystem, -0.3)));
    // coralJoystick.button(1).toggleOnTrue(new InstantCommand(() -> coralSubsystem.pidSetPosition(CoralPivotPositions.L1))).toggleOnFalse(new InstantCommand(() -> coralSubsystem.stopPivotMotor()));
    coralJoystick.button(1).toggleOnTrue(new runMotor(coralSubsystem, 0.1));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
