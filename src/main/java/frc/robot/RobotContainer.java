// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.commands.AutoReefAlignLeft;
// import frc.robot.commands.AutoReefAlignRight;
import frc.robot.commands.BargeScore;
import frc.robot.commands.CoralStationIntake;
import frc.robot.commands.ManualClimb;
import frc.robot.subsystems.CoralAlgaeSubsystem;
import frc.robot.subsystems.CoralAlgaeSubsystem.CoralPivotPositions;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPositions;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ResetElevator;
import frc.robot.commands.ScoreAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoringPosition;
import frc.robot.commands.Stow;
import frc.robot.commands.CoralAlgaeCommands.IntakeAlgae;
import frc.robot.commands.CoralAlgaeCommands.OuttakeAlgae;
import frc.robot.commands.CoralAlgaeCommands.OuttakeCoral;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.AutoCommands.AutoCoralStationIntake;
import frc.robot.commands.AutoCommands.AutoScoringPosition;
import frc.robot.commands.AutoCommands.AutoStow;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
    SendableChooser<Command> autoChooser = new SendableChooser<>();
    Field2d field = new Field2d();

//     public final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final CoralAlgaeSubsystem coralAlgaeSubsystem = new CoralAlgaeSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    // private CommandJoystick testJoystick = new CommandJoystick(1);
    // private CommandJoystick testCoralJoystick = new CommandJoystick(2);
    private CommandJoystick buttonBoard = new CommandJoystick(1);
    final CommandXboxController driverController = new CommandXboxController(0);
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    private int driverLeftAlign = 1;
    private int driverRightAlign = 2;

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
    private int stowButton = 4;

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
            () -> driverController.getLeftY() * 1,
            () -> driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> -driverController.getRawAxis(2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(1.2)
            .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
            .withControllerHeadingAxis(driverController::getRightX,
                    driverController::getRightY)
            .headingWhile(true);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = swerveSubsystem.driveFieldOriented(driveDirectAngle);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

    Command driveSetpointGen = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

    SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX())
            .withControllerRotationAxis(() -> driverController.getRawAxis(2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                    driverController.getRawAxis(
                            2) * Math.PI)
                    * (Math.PI * 2),
                    () -> Math.cos(
                            driverController.getRawAxis(
                                    2) * Math.PI)
                            *
                            (Math.PI * 2))
            .headingWhile(true);

    Command driveFieldOrientedDirectAngleSim = swerveSubsystem.driveFieldOriented(driveDirectAngleSim);

    Command driveFieldOrientedAnglularVelocitySim = swerveSubsystem.driveFieldOriented(driveAngularVelocitySim);

    Command driveSetpointGenSim = swerveSubsystem.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        elevatorSubsystem.zeroElevatorPosition();
        NamedCommands.registerCommand("Elevator to L4",
                new SequentialCommandGroup(
                        new AutoScoringPosition(coralAlgaeSubsystem, CoralPivotPositions.L4, elevatorSubsystem,
                                ElevatorPositions.L4),
                        new InstantCommand(() -> coralAlgaeSubsystem.setPIDPosition(CoralPivotPositions.L4)),
                        new WaitUntilCommand(() -> elevatorSubsystem.encoderToInch() > 85)));
        NamedCommands.registerCommand("Elevator to L3",
                new AutoScoringPosition(coralAlgaeSubsystem, CoralPivotPositions.L3, elevatorSubsystem,
                        ElevatorPositions.L3));
        NamedCommands.registerCommand("Elevator to L2",
                new AutoScoringPosition(coralAlgaeSubsystem, CoralPivotPositions.L2, elevatorSubsystem,
                        ElevatorPositions.L2));
        NamedCommands.registerCommand("Elevator to L1",
                new AutoScoringPosition(coralAlgaeSubsystem, CoralPivotPositions.L1, elevatorSubsystem,
                        ElevatorPositions.L1));
        NamedCommands.registerCommand("Coral Station Intake",
                new AutoCoralStationIntake(coralAlgaeSubsystem, elevatorSubsystem));
        NamedCommands.registerCommand("Stow", new AutoStow(coralAlgaeSubsystem, elevatorSubsystem));
        NamedCommands.registerCommand("Score Coral", new OuttakeCoral(coralAlgaeSubsystem, 1.0).withTimeout(1));
        
        autoChooser.addOption("Processor Side 2.5", swerveSubsystem.getAutonomousCommand("2.5 Processor Side"));
        autoChooser.addOption("NOT Processor Side 2.5", swerveSubsystem.getAutonomousCommand("2.5 NoProcessor Side"));
        autoChooser.addOption("Back up", swerveSubsystem.driveCommand(() -> 0, () -> -0.5, () -> 0).withTimeout(1));
        SmartDashboard.putData("Autonomous Routines", autoChooser);

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

        new Trigger(() -> buttonBoard.getY() > 0).whileTrue(new ManualClimb(climberSubsystem, () -> 0.5));

        // new Trigger(() -> buttonBoard.getX() > 0.5).onTrue(new InstantCommand(() -> elevatorSubsystem.addSubtractManualOffset(elevatorSubsystem.inchToEncoderConverter(1))));
        // new Trigger(() -> buttonBoard.getX() < 0.5).onTrue(new InstantCommand(() -> elevatorSubsystem.addSubtractManualOffset(-elevatorSubsystem.inchToEncoderConverter(1))));
        buttonBoard.button(rightOffsetButton).onTrue(new InstantCommand(() -> elevatorSubsystem.addSubtractManualOffset(elevatorSubsystem.inchToEncoderConverter(1))));
        buttonBoard.button(leftOffsetButton).onTrue(new InstantCommand(() -> elevatorSubsystem.addSubtractManualOffset(-elevatorSubsystem.inchToEncoderConverter(1))));

        buttonBoard.button(coralAlgaeSelector)
                .onTrue(new InstantCommand(
                        () -> coralAlgaeSubsystem.setCoralIntakeMode(!coralAlgaeSubsystem.coralIntaking)))
                .debounce(1);

        buttonBoard.button(L4Button).onTrue(new ConditionalCommand(
                new ScoringPosition(coralAlgaeSubsystem, CoralPivotPositions.L4, elevatorSubsystem,
                        ElevatorPositions.L4),
                new BargeScore(coralAlgaeSubsystem, elevatorSubsystem),
                () -> coralAlgaeSubsystem.coralIntaking));

        buttonBoard.button(L3Button).onTrue(new ConditionalCommand(
                new ScoringPosition(coralAlgaeSubsystem, CoralPivotPositions.L3, elevatorSubsystem,
                        ElevatorPositions.L3),
                new ScoringPosition(coralAlgaeSubsystem, CoralPivotPositions.AlgaeReef, elevatorSubsystem,
                        ElevatorPositions.AlgaeReefHigh),
                () -> coralAlgaeSubsystem.coralIntaking));

        buttonBoard.button(L2Button).onTrue(new ConditionalCommand(
                new ScoringPosition(coralAlgaeSubsystem, CoralPivotPositions.L2, elevatorSubsystem,
                        ElevatorPositions.L2),
                new ScoringPosition(coralAlgaeSubsystem, CoralPivotPositions.AlgaeReef, elevatorSubsystem,
                        ElevatorPositions.AlgaeReefLow),
                () -> coralAlgaeSubsystem.coralIntaking));

        buttonBoard.button(L1Button).onTrue(new ConditionalCommand(
                new ScoringPosition(coralAlgaeSubsystem, CoralPivotPositions.L2, elevatorSubsystem,
                        ElevatorPositions.L2),
                new ScoringPosition(coralAlgaeSubsystem, CoralPivotPositions.AlgaeReef,
                        elevatorSubsystem, ElevatorPositions.Processor),
                        () -> coralAlgaeSubsystem.coralIntaking));

        buttonBoard.button(fireButton).and(() -> coralAlgaeSubsystem.coralIntaking)
                .onTrue(new ScoreCoral(elevatorSubsystem, coralAlgaeSubsystem));

        buttonBoard.button(fireButton).and(() -> !coralAlgaeSubsystem.coralIntaking)
                .whileFalse(new OuttakeAlgae(coralAlgaeSubsystem, 0))
                .whileTrue(new OuttakeAlgae(coralAlgaeSubsystem, -0.75))
                .onFalse(new SequentialCommandGroup(new WaitCommand(1), new Stow(coralAlgaeSubsystem, elevatorSubsystem)));

        buttonBoard.button(coralIntakeButton).and(() -> coralAlgaeSubsystem.coralIntaking)
                .onTrue(new CoralStationIntake(coralAlgaeSubsystem, elevatorSubsystem));

        buttonBoard.button(coralIntakeButton).and(() -> !coralAlgaeSubsystem.coralIntaking)
                .whileTrue(new IntakeAlgae(coralAlgaeSubsystem, 1))
                .whileFalse(new IntakeAlgae(coralAlgaeSubsystem, 0));

        buttonBoard.button(stowButton).onTrue(new Stow(coralAlgaeSubsystem, elevatorSubsystem)).debounce(0.3)
                .whileTrue(new ResetElevator(elevatorSubsystem, coralAlgaeSubsystem));


        driverController.button(driverRightAlign)
                .onTrue(new DeferredCommand(() -> swerveSubsystem.rightAutoAlign(), Set.of(swerveSubsystem)));
        driverController.button(driverLeftAlign)
                .onTrue(new DeferredCommand(() -> swerveSubsystem.leftAutoAlign(), Set.of(swerveSubsystem)));

        // swerve logic
        // (Condition) ? Return-On-True : Return-on-False
        swerveSubsystem.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocitySim);

        if (Robot.isSimulation()) {
            // driverController.start().onTrue(Commands.runOnce(() ->
            // swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            driverController.button(1).whileTrue(swerveSubsystem.sysIdDriveMotorCommand());

        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        swerveSubsystem.setMotorBrake(brake);
    }

    public void updateDashboard() {
        SmartDashboard.putBoolean("Coral Mode", coralAlgaeSubsystem.coralIntaking);
        // field.setRobotPose(Vision.getAprilTagPose(18, new Transform2d(0, 0, new
        // Rotation2d(0))));
        // SmartDashboard.putBoolean("coral score is true",
        // coralAlgaeSubsystem.pidWithinBounds(CoralPivotPositions.CoralStation.L4,
        // 0.05, 1));
        SmartDashboard.putNumber("Back motor percent", elevatorSubsystem.getBackElevatorSpeed());
        SmartDashboard.putNumber("Coral Pivot Position", coralAlgaeSubsystem.getPivotPosition());
        // SmartDashboard.putNumber("Front Elevator Position",
        // elevatorSubsystem.getFrontElevatorMotorEncoder());
        // SmartDashboard.putNumber("Back Elevator Position",
        // elevatorSubsystem.getBackElevatorMotorEncoder());
        SmartDashboard.putBoolean("is Front Limit Switch enabled", elevatorSubsystem.getFrontElevatorLimitSwitch());
        SmartDashboard.putBoolean("is Back Limit Switch enabled", elevatorSubsystem.getBackElevatorLimitSwitch());
        SmartDashboard.putNumber("Elevator Position Enc", elevatorSubsystem.getElevatorPosition());
        SmartDashboard.putNumber("Elevator Position Inch", elevatorSubsystem.encoderToInch());
        // SmartDashboard.putNumber("Difference in velocity",
        // elevatorSubsystem.differenceInVelocity());
        SmartDashboard.putNumber("Elevator Target Position", elevatorSubsystem.getElevatorTarget().getValue());
        SmartDashboard.putBoolean("coral break beam", coralAlgaeSubsystem.getCoralSensor());

        SmartDashboard.putNumber("left climber position", climberSubsystem.leftClimbMotorPosition());
        SmartDashboard.putNumber("right climber position", climberSubsystem.rightClimbMotorPosition());

        SmartDashboard.putNumber("Intake Current", coralAlgaeSubsystem.getAlgaeCurrent());
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    }

    public void resetPIDControllers() {
        elevatorSubsystem.resetPIDController();
        coralAlgaeSubsystem.resetPIDController();
    }

    public boolean isCoralIntakeMode(){
        return coralAlgaeSubsystem.coralIntaking;
    }
}