package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;

public class CoralAlgaeSubsystem extends SubsystemBase {

    private SparkLimitSwitch coralSensor;
    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    private SparkAbsoluteEncoder throughBoreEncoder;

    private SparkClosedLoopController pivotMotorPID;
    private SparkMaxConfig pivotMotorConfig;
    private SparkMaxConfig intakeMotorConfig;
    public boolean coralIntaking = true;
    //.7199
    public enum CoralPivotPositions {
        // increases moving towards the front
        L1(0.537),
        L2(0.72),
        L3(0.72),
        L4(0.743 + 0.008),
        AlgaeReef(0.5),
        Barge(0.4),
        Stow(0.256),
        CoralStation(.215),
        MinimumAngle(.5),
        MaximumAngle(.5);
        private final double value;

        CoralPivotPositions(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

    }

    public CoralAlgaeSubsystem() {
        intakeMotor = new SparkMax(48, MotorType.kBrushed);
        pivotMotor = new SparkMax(47, MotorType.kBrushless);
        coralSensor = intakeMotor.getForwardLimitSwitch();
        intakeMotorConfig = new SparkMaxConfig();
        pivotMotorConfig = new SparkMaxConfig();
        pivotMotorPID = pivotMotor.getClosedLoopController();
        throughBoreEncoder = pivotMotor.getAbsoluteEncoder();

        intakeMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);

        intakeMotorConfig.limitSwitch
            .forwardLimitSwitchEnabled(false);

        pivotMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(7)
                .i(0)
                .d(3)
                .maxOutput(0.75)
                .minOutput(-0.75);

        
        pivotMotorConfig
                .inverted(true)
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake);
        
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPivotMotorSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void stopPivotMotor() {
        pivotMotor.stopMotor();
    }

    public double getPivotPosition() {
        return throughBoreEncoder.getPosition();
    }

    public double getPivotVelocity() {
        return throughBoreEncoder.getVelocity();
    }

    public void setIntakeMotorSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntakeMotor() {
        intakeMotor.stopMotor();
    }

    public boolean getCoralSensor(){
        return coralSensor.isPressed();
    }

    public void setPIDPosition(CoralPivotPositions position) {
        pivotMotorPID.setReference(position.getValue(), ControlType.kPosition);
    }

    // public boolean pidWithinBounds(CoralPivotPositions position, double positionTolerance, double velocityTolerance) {
    //     double upperbound = position.getValue() + positionTolerance;
    //     double lowerbound = position.getValue() - positionTolerance;
    //     boolean withinPosition = getPivotPosition() <= upperbound && getPivotPosition() >= lowerbound;
    //     boolean withinVelocity = getPivotVelocity() <= Math.abs(velocityTolerance) && getPivotVelocity() >= -Math.abs(velocityTolerance);
    //     return (withinPosition && withinVelocity);
    // }

    public boolean pidWithinBounds(CoralPivotPositions position, double positionTolerance, double velocityTolerance) {
        double upperbound = position.getValue() + positionTolerance;
        double lowerbound = position.getValue() - positionTolerance;
        boolean withinPosition = getPivotPosition() <= upperbound && getPivotPosition() >= lowerbound;
        boolean withinVelocity = getPivotVelocity() <= Math.abs(velocityTolerance) && getPivotVelocity() >= -Math.abs(velocityTolerance);
        return (withinPosition && withinVelocity);
    }

    public void resetPIDController(){
        pivotMotor.set(0);
    }

    public void setCoralIntakeMode(boolean coralIntake){
        coralIntaking = coralIntake;
    }

    public double getAlgaeCurrent(){
        return intakeMotor.getOutputCurrent();
    }
}
