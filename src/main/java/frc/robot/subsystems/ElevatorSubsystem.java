package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
        L1(10),
        L2(41),
        L3(60),
        L4(90.495417),
        Barge(90.495417),
        AlgaeReefHigh(45.5),
        AlgaeReefLow(30),
        Processor(20),
        Stow(32),
        CoralStation(34),
        MaxHeight(92),
        MinimumHeight(8);

        private final double value;

        ElevatorPositions(double value) {
            this.value = value;
        }

        public double getValue() {
            // return .000130994x^{4}-.0067331x^{3}+.118148x^{2}+2.30416x+8;
            return 
            0.000000723783*Math.pow(value, 4) 
            -0.000158195*Math.pow(value, 3) 
                +0.0113236*Math.pow(value, 2) 
                +0.0272627*value 
                +0;
        }
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
                .forwardSoftLimit(ElevatorPositions.MaxHeight.getValue())
                .forwardSoftLimitEnabled(true);

        backElevatorMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(40);

        backElevatorMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .p(16)
                .d(0)
                .outputRange(-1, 1);

        // front motor is follower
        frontElevatorMotorConfig
                .inverted(false)
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake)
                .follow(backElevatorMotor.getDeviceId());

        backElevatorMotor.configure(backElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        frontElevatorMotor.configure(frontElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public double getBackElevatorSpeed(){
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

    public double getElevatorVelocity(){
        return throughBoreEncoder.getVelocity();
    }

    public void setElevatorSpeed(double speed) {
        backElevatorMotor.set(speed);
    }

    public void setPIDPosition(ElevatorPositions position) {
        targetElevatorPosition = position;
        backElevatorMotorPID.setReference(position.getValue(), ControlType.kPosition);
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

    public double differenceInVelocity() {
        return backElevatorMotor.getEncoder().getVelocity() - frontElevatorMotor.getEncoder().getVelocity();
    }

    public boolean pidWithinBounds(ElevatorPositions position, double positionTolerance, double velocityTolerance) {
        double upperbound = position.getValue() + positionTolerance;
        double lowerbound = position.getValue() - positionTolerance;
        boolean withinPosition = getElevatorPosition() <= upperbound && getElevatorPosition() >= lowerbound;
        boolean withinVelocity = getElevatorVelocity() <= Math.abs(velocityTolerance) && getElevatorVelocity() >= -Math.abs(velocityTolerance);
        return (withinPosition && withinVelocity);
    }

    public void zeroElevatorPosition() {
        backElevatorMotor.getEncoder().setPosition(0);
    }

    public void resetPIDController(){
        backElevatorMotor.set(0);
        frontElevatorMotor.set(0);
    }
    
    public double encoderToInch(){
        // return 2.98857 * getElevatorPosition() + 7.62804;
        return 
            0.000130884*Math.pow(getElevatorPosition(), 4) 
            -0.0067331*Math.pow(getElevatorPosition(), 3) 
            +0.118148*Math.pow(getElevatorPosition(), 2) 
            +2.30416*getElevatorPosition() 
            +8;
    }

    public double inchToEncoder(){
        return 
            0.000000723783*Math.pow(getElevatorPosition(), 4) 
            -0.000158195*Math.pow(getElevatorPosition(), 3) 
                +0.0113236*Math.pow(getElevatorPosition(), 2) 
                +0.0272627*getElevatorPosition() 
                +0;
    }

    @Override
    public void periodic() {

        // if (getFrontElevatorLimitSwitch() && backElevatorMotor.get() < 0) {
        //     backElevatorMotor.stopMotor();
        // }

        if (getFrontElevatorLimitSwitch() && getElevatorPosition() != 0) {
            throughBoreEncoder.setPosition(0);
        }

        
    }
}