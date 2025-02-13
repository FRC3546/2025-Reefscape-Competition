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

public class CoralSubsystem extends SubsystemBase {

    private SparkLimitSwitch coralSensor;
    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    private SparkAbsoluteEncoder throughBoreEncoder;

    private SparkClosedLoopController pivotMotorPID;
    private SparkMaxConfig pivotMotorConfig;
    private SparkMaxConfig intakeMotorConfig;

    public enum CoralPivotPositions {
        // increases moving towards the front
        L1(0.2),
        L2(0),
        L3(0),
        L4(0),
        Stow(0.94),
        CoralStation(0),
        MinimumAngle(.14),
        MaximumAngle(0);

        private final double value;

        CoralPivotPositions(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

    }

    public CoralSubsystem() {
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

        pivotMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(4)
                .d(0)
                .outputRange(-.5, .5);
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

    public boolean pidWithinBounds(CoralPivotPositions position, double tolerance) {
        double upperbound = position.getValue() + tolerance;
        double lowerbound = position.getValue() - tolerance;
        return (getPivotPosition() <= upperbound && getPivotPosition() >= lowerbound);
    }
}
