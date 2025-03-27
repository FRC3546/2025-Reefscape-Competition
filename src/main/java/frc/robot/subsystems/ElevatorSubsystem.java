package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem extends SubsystemBase {

    public double manualOffset = 0;
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
        // L1(10),
        // L2(45),
        // L3(64),
        // L4(91.25),
        // Barge(90.495417),
        // AlgaeReefHigh(54.5),
        // AlgaeReefLow(39),
        // Processor(23),
        // Stow(32),
        // CoralStation(30.5),
        // MaxHeight(94),
        // MinimumHeight(8);
        L1(2.04),
        L2(4.75),
        L3(6.75),
        L4(10.45),
        Barge(10.45),
        AlgaeReefHigh(5.15),
        AlgaeReefLow(3.2),
        Processor(3.5),
        Stow(4),
        CoralStation(3.18),
        MaxHeight(10.45),
        MinimumHeight(0);

        private final double value;

        ElevatorPositions(double value) {
            this.value = value;
        }

        public double getValueRotations() {
            // return 0.329911 * value - 1.92945;
            return value;
        }

        // public double getValueInches() {
        // return value;
        // }
    }

    public ElevatorSubsystem() {
        // elevator direction good
        backElevatorLimitSwitch = new DigitalInput(0);
        frontElevatorLimitSwitch = new DigitalInput(1);
        targetElevatorPosition = ElevatorPositions.Stow;
        backElevatorMotor = new SparkMax(17, MotorType.kBrushless);
        frontElevatorMotor = new SparkMax(18, MotorType.kBrushless);
        backElevatorMotorPID = backElevatorMotor.getClosedLoopController();
        backElevatorMotorConfig = new SparkMaxConfig();
        frontElevatorMotorConfig = new SparkMaxConfig();
        throughBoreEncoder = backElevatorMotor.getAlternateEncoder();

        backElevatorMotorConfig.alternateEncoder
                .countsPerRevolution(8192)
                .setSparkMaxDataPortConfig()
                .inverted(true);

        backElevatorMotorConfig.softLimit
                .forwardSoftLimit(ElevatorPositions.MaxHeight.getValueRotations())
                .forwardSoftLimitEnabled(true);

        backElevatorMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(60);

        // kSlot0 = down PID
        // kSlot1 = up PID
        backElevatorMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .p(0.85, ClosedLoopSlot.kSlot0)
                .i(0.0000000000000015, ClosedLoopSlot.kSlot0)
                .d(0, ClosedLoopSlot.kSlot0)
                .iMaxAccum(0.010000000, ClosedLoopSlot.kSlot0)
                .outputRange(-0.8, 1, ClosedLoopSlot.kSlot0)

                .p(1.5, ClosedLoopSlot.kSlot1)
                .i(0.0000000000000015, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .iMaxAccum(0.010000000, ClosedLoopSlot.kSlot1)
                .outputRange(-0.8, 1, ClosedLoopSlot.kSlot1);
        ;

        // front motor is follower
        frontElevatorMotorConfig
                .inverted(false)
                .smartCurrentLimit(60)
                .idleMode(IdleMode.kBrake)
                .follow(backElevatorMotor.getDeviceId());

        backElevatorMotor.configure(backElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        frontElevatorMotor.configure(frontElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public double getBackElevatorSpeed() {
        return backElevatorMotor.get();
    }

    public double getBackElevatorMotorEncoder() {
        return backElevatorMotor.getEncoder().getPosition();
    }

    public double getFrontElevatorMotorEncoder() {
        return backElevatorMotor.getEncoder().getPosition();
    }

    public double getElevatorPosition() {
        return throughBoreEncoder.getPosition();
    }

    public double getElevatorVelocity() {
        return throughBoreEncoder.getVelocity();
    }

    public ElevatorPositions getTargetElevatorPosition() {
        return targetElevatorPosition;
    }

    public void setElevatorSpeed(double speed) {
        backElevatorMotor.set(speed);
    }

    public void setPIDPosition(ElevatorPositions position) {
        targetElevatorPosition = position;
        if (getElevatorPosition() < position.getValueRotations()) {
            backElevatorMotorPID.setReference(position.getValueRotations() + getManualOffset(), ControlType.kPosition,
                    ClosedLoopSlot.kSlot1);
        } else if (getElevatorPosition() > position.getValueRotations()) {
            backElevatorMotorPID.setReference(position.getValueRotations() + getManualOffset(), ControlType.kPosition,
                    ClosedLoopSlot.kSlot0);
        } else {
            backElevatorMotorPID.setReference(position.getValueRotations() + getManualOffset(), ControlType.kPosition,
                    ClosedLoopSlot.kSlot0);
        }

    }

    public void bumpPIDPosition(double offset) {
        backElevatorMotorPID.setReference(getElevatorPosition() + offset, ControlType.kPosition);
    }

    public ElevatorPositions getElevatorTarget() {
        return targetElevatorPosition;
    }

    public boolean getBackElevatorLimitSwitch() {
        return backElevatorLimitSwitch.get();
    }

    public boolean getFrontElevatorLimitSwitch() {
        return frontElevatorLimitSwitch.get();
    }

    public void setTargetPosition(ElevatorPositions elevatorPosition) {
        targetElevatorPosition = elevatorPosition;
    }

    public void zeroElevatorPosition() {
        System.out.println("Resetting Elevator Encoder...");
        throughBoreEncoder.setPosition(0);
        System.out.println("New Position: " + throughBoreEncoder.getPosition());
    }

    public void resetPIDController() {
        backElevatorMotor.set(0);
        frontElevatorMotor.set(0);
    }

    // public double encoderToInch() {
    // return 2.91627 * getElevatorPosition() + 8;
    // }

    // public double inchToEncoder() {
    // return 0.000000723783 * Math.pow(getElevatorPosition(), 4)
    // - 0.000158195 * Math.pow(getElevatorPosition(), 3)
    // + 0.0113236 * Math.pow(getElevatorPosition(), 2)
    // + 0.0272627 * getElevatorPosition()
    // + 0;
    // }

    // public double inchToEncoderConverter(double inches) {
    // return 0.329911 * inches - 1.92945;
    // }

    public void setManualOffset(double manualOffset) {
        this.manualOffset = manualOffset;
    }

    public void addSubtractManualOffset(double addSubstract) {
        manualOffset += addSubstract;
    }

    public double getManualOffset() {
        return manualOffset;
    }

    public double getFrontElevatorCurrent() {
        return frontElevatorMotor.getOutputCurrent();
    }

    public double getBackElevatorCurrent() {
        return backElevatorMotor.getOutputCurrent();
    }
}