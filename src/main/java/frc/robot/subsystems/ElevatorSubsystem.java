package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorSubsystem extends SubsystemBase {

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
        MaxHeight(0),
        MinimumHeight(0);

        private final double value;

        ElevatorPositions(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    public ElevatorSubsystem() {
        targetElevatorPosition = ElevatorPositions.Stow;
        backElevatorMotor = new SparkMax(0, MotorType.kBrushless);
        frontElevatorMotor = new SparkMax(0, MotorType.kBrushless);
        backElevatorMotorPID = backElevatorMotor.getClosedLoopController();
        backElevatorMotorConfig = new SparkMaxConfig();
        frontElevatorMotorConfig = new SparkMaxConfig();
        throughBoreEncoder = backElevatorMotor.getAlternateEncoder();

        backElevatorMotorConfig.softLimit
            .forwardSoftLimit(ElevatorPositions.MaxHeight.getValue())
            .reverseSoftLimit(ElevatorPositions.MinimumHeight.getValue())
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);

        backElevatorMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .p(0.001)
                .d(0)
                .outputRange(-1, 1);
        
                backElevatorMotor.configure(backElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // right motor is follower
        frontElevatorMotorConfig.follow(backElevatorMotor.getDeviceId(), false);
        frontElevatorMotor.configure(frontElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public double getbackElevatorMotorEncoder() {
        return backElevatorMotor.getEncoder().getPosition();
    }

    public double getfrontElevatorMotorEncoder() {
        return backElevatorMotor.getEncoder().getPosition();
    }

    public double getElevatorPosition() {
        return throughBoreEncoder.getPosition();
    }

    public double getElevatorVelocity() {
        return throughBoreEncoder.getVelocity();
    }

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
}