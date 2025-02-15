// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.hal.HAL.SimPeriodicAfterCallback;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CoralStationIntake;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ManualClimb;
import frc.robot.commands.ManualCoral;
import frc.robot.commands.ManualCoralAlignment;
import frc.robot.commands.ManualElevator;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.CoralPivotPositions;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.OuttakeCoral;
import frc.robot.commands.ResetElevator;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoringPosition;
import frc.robot.commands.Stow;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private CommandJoystick testJoystick = new CommandJoystick(1);
  // private CommandJoystick testCoralJoystick = new CommandJoystick(2);
  private CommandJoystick buttonBoard = new CommandJoystick(2);
  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  private int L1Button = 9;
  private int L2Button = 8;
  private int L3Button = 5;
  private int L4Button = 1;

  private int disableAutoButton = 2;

  private int leftOffsetButton = 10;
  private int rightOffsetButton = 11;
  private int coralIntakeButton = 6;
  private int fireButton = 7;
  private int coralAlgaeSelector = 3;
  private int zeroButton = 4;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * 1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(1.2)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) * Math.PI)
          * (Math.PI * 2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) * Math.PI)
              *
              (Math.PI * 2))
      .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    elevatorSubsystem.zeroElevatorPosition();
    // elevatorSubsystem.setDefaultCommand(new ManualElevator(elevatorSubsystem, () -> (testJoystick.getY()) / 0.5));
    // climberSubsystem.setDefaultCommand(new ManualClimb(climberSubsystem, () -> testJoystick.getY()));

    // coralSubsystem.setDefaultCommand(new ManualCoral(coralSubsystem, () ->
    // (testJoystick.getY()/4)));
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    buttonBoard.button(L4Button).onTrue(new ScoringPosition(coralSubsystem, CoralPivotPositions.L4, elevatorSubsystem, ElevatorPositions.L4));
    buttonBoard.button(L3Button).onTrue(new ScoringPosition(coralSubsystem, CoralPivotPositions.L3, elevatorSubsystem, ElevatorPositions.L3));
    buttonBoard.button(L2Button).onTrue(new ScoringPosition(coralSubsystem, CoralPivotPositions.L2, elevatorSubsystem, ElevatorPositions.L2));
    buttonBoard.button(L1Button).onTrue(new ScoringPosition(coralSubsystem, CoralPivotPositions.L1, elevatorSubsystem, ElevatorPositions.L1));

    buttonBoard.button(rightOffsetButton).whileTrue(new ManualCoralAlignment(drivebase, -0.4));
    buttonBoard.button(leftOffsetButton).whileTrue(new ManualCoralAlignment(drivebase, 0.4));
    buttonBoard.button(fireButton).onTrue(new OuttakeCoral(coralSubsystem, 0.75));
    buttonBoard.button(coralIntakeButton).onTrue(new CoralStationIntake(coralSubsystem, elevatorSubsystem));
    buttonBoard.button(zeroButton).onTrue(new ResetElevator(elevatorSubsystem, coralSubsystem));

    

    // swerve logic
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocitySim);

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("4 Coral Red Barge");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void updateDashboard() {
    SmartDashboard.putBoolean("coral score is true", coralSubsystem.pidWithinBounds(CoralPivotPositions.CoralStation.L4, 0.05, 1));
    SmartDashboard.putNumber("Back motor percent", elevatorSubsystem.getBackElevatorSpeed());
    SmartDashboard.putNumber("Coral Pivot Position", coralSubsystem.getPivotPosition());
    SmartDashboard.putNumber("Front Elevator Position", elevatorSubsystem.getFrontElevatorMotorEncoder());
    SmartDashboard.putNumber("Back Elevator Position", elevatorSubsystem.getBackElevatorMotorEncoder());
    SmartDashboard.putBoolean("is Front Limit Switch enabled", elevatorSubsystem.getFrontElevatorLimitSwitch());
    SmartDashboard.putBoolean("is Back Limit Switch enabled", elevatorSubsystem.getBackElevatorLimitSwitch());
    SmartDashboard.putNumber("Elevator Position Enc", elevatorSubsystem.getElevatorPosition());
    SmartDashboard.putNumber("Elevator Position Inch", elevatorSubsystem.encoderToInch());
    SmartDashboard.putNumber("Difference in velocity", elevatorSubsystem.differenceInVelocity());
    SmartDashboard.putNumber("Elevator Target Position", elevatorSubsystem.getElevatorTarget().getValue());
    SmartDashboard.putBoolean("coral break beam", coralSubsystem.getCoralSensor());

    SmartDashboard.putNumber("left climber position", climberSubsystem.leftClimbMotorPosition());
    SmartDashboard.putNumber("right climber position", climberSubsystem.rightClimbMotorPosition());

    // elevatorSubsystem.getElevatorPosition());
  }

  public void resetPIDControllers(){
    elevatorSubsystem.resetPIDController();
    coralSubsystem.resetPIDController();
  }
}