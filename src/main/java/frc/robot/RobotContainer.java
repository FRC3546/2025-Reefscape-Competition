// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ledTestCommand;
import frc.robot.subsystems.CANdleSubsystem;

public class RobotContainer {
  private final CANdleSubsystem leds = new CANdleSubsystem();
  private final Joystick testJoystick = new Joystick(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Trigger testTrigger = new Trigger(() -> testJoystick.getRawButton(1));
    Trigger testButton = new Trigger(() -> testJoystick.getRawButton(2));
    Trigger testButton2 = new Trigger(() -> testJoystick.getRawButton(3));
    testTrigger.onTrue(new InstantCommand(() -> leds.solid(CANdleSubsystem.Color.red)));
    testButton.onTrue(new InstantCommand(() -> leds.solid(CANdleSubsystem.Color.yellow)));
    testButton2.onTrue(new ledTestCommand(leds));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
