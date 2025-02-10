package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralSubsystem.CoralPivotPositions;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem extends SubsystemBase {

    private DigitalInput frontElevatorLimitSwitch;
    private DigitalInput backElevatorLimitSwitch;
    private SparkMax backElevatorMotor;
    private SparkMax frontElevatorMotor;
    private RelativeEncoder throughBoreEncoder;
    private SparkClosedLoopController backElevatorMotorPID;
    private SparkMaxConfig backElevatorMotorConfig;
    private SparkMaxConfig frontElevatorMotorConfig;
    private ElevatorPositions targetElevatorPosition;

    public enum ElevatorPositions {
        L1(0),
        L2(0),
        L3(0),
        L4(0),
        Barge(0),
        AlgaeReefHigh(0),
        AlgaeReefLow(0),
        Processor(0),
        Stow(0),
        CoralStation(0),
        MaxHeight(-1250),
        MinimumHeight(0);
        //MAX = -1300
        private final double value;

        ElevatorPositions(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    public ElevatorSubsystem() {
        // elevator direction good
        backElevatorLimitSwitch = new DigitalInput(0);
        frontElevatorLimitSwitch = new DigitalInput(1);
        targetElevatorPosition = ElevatorPositions.Stow;
        backElevatorMotor = new SparkMax(50, MotorType.kBrushless);
        frontElevatorMotor = new SparkMax(51, MotorType.kBrushless);
        backElevatorMotorPID = backElevatorMotor.getClosedLoopController();
        backElevatorMotorConfig = new SparkMaxConfig();
        frontElevatorMotorConfig = new SparkMaxConfig();
        // throughBoreEncoder = backElevatorMotor.getAlternateEncoder();

        backElevatorMotorConfig.softLimit
        .reverseSoftLimit(ElevatorPositions.MaxHeight.getValue())
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(.1)
        .forwardSoftLimitEnabled(false);

        backElevatorMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(40);

        backElevatorMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(1)
                .d(0)
                .outputRange(-1, 1);

        backElevatorMotorConfig.closedLoop
                .p(0.001)
                .d(0)
                .outputRange(-1, 1);

        // front motor is follower
        frontElevatorMotorConfig
                .inverted(true)
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake)
                .follow(backElevatorMotor.getDeviceId());

        backElevatorMotor.configure(backElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        frontElevatorMotor.configure(frontElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public double getBackElevatorMotorEncoder() {
        return backElevatorMotor.getEncoder().getPosition();
    }

    public double getFrontElevatorMotorEncoder() {
        return backElevatorMotor.getEncoder().getPosition();
    }

    public double getElevatorPosition() {
        return backElevatorMotor.getEncoder().getPosition();
    }

    // public double getElevatorPosition() {
    // return throughBoreEncoder.getPosition();
    // }

    // public double getElevatorVelocity() {
    // return throughBoreEncoder.getVelocity();
    // }

    public void setElevatorSpeed(double speed) {
        backElevatorMotor.set(speed);
    }

    public void pidSetPosition(ElevatorPositions position) {
        targetElevatorPosition = position;
        backElevatorMotorPID.setReference(position.getValue(), ControlType.kPosition);
    }

    public ElevatorPositions getElevatorTarget() {
        return targetElevatorPosition;
    }

    public boolean getBackElevatorLimitSwitch(){
        return backElevatorLimitSwitch.get();
    }

    public boolean getFrontElevatorLimitSwitch(){
        return frontElevatorLimitSwitch.get();
    }

    public double differenceInVelocity(){
        return backElevatorMotor.getEncoder().getVelocity() - frontElevatorMotor.getEncoder().getVelocity();
    }

    public boolean pidWithinBounds(ElevatorPositions position, double tolerance) {
        double upperbound = position.getValue() + tolerance;
        double lowerbound = position.getValue() - tolerance;
        return (getElevatorPosition() <= upperbound && getElevatorPosition() >= lowerbound);
    }

    public void zeroElevatorPosition(){
        backElevatorMotor.getEncoder().setPosition(0);
    }

   @Override
   public void periodic(){
    if(getFrontElevatorLimitSwitch() && getElevatorPosition() != 0){
        backElevatorMotor.getEncoder().setPosition(0);
        frontElevatorMotor.getEncoder().setPosition(0);
    }

    if(getFrontElevatorLimitSwitch() && backElevatorMotor.get() > 0){
        backElevatorMotor.stopMotor();
    }
   }
}