package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeSubsystem extends SubsystemBase {

    private SparkMax pivotMotor;
    private SparkMax leftIntakeMotor;
    private SparkMax rightIntakeMotor;
    private SparkAbsoluteEncoder throughBoreEncoder;

    private SparkClosedLoopController pivotMotorPID;
    private SparkMaxConfig pivotMotorConfig;

    private SparkMaxConfig leftIntakeMotorConfig;
    private SparkMaxConfig rightIntakeMotorConfig;

    public enum PivotPositions {
        Ground(0),
        Processor(0),
        Barge(0),
        Stowed(0),
        Reef(0);

        private final double value;

        PivotPositions(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

    }

    public AlgaeSubsystem() {
        pivotMotor = new SparkMax(81, MotorType.kBrushless);
        leftIntakeMotor = new SparkMax(82, MotorType.kBrushless);
        rightIntakeMotor = new SparkMax(83, MotorType.kBrushless);

        leftIntakeMotorConfig = new SparkMaxConfig();
        leftIntakeMotorConfig.smartCurrentLimit(20);
        leftIntakeMotor.configure(leftIntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        rightIntakeMotorConfig = new SparkMaxConfig();
        rightIntakeMotorConfig.follow(leftIntakeMotor.getDeviceId(), false);
        rightIntakeMotorConfig.smartCurrentLimit(20);
        rightIntakeMotor.configure(rightIntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        pivotMotorPID = pivotMotor.getClosedLoopController();
        pivotMotorConfig = new SparkMaxConfig();
        throughBoreEncoder = pivotMotor.getAbsoluteEncoder();

        pivotMotorConfig.closedLoop
                .p(0.001)
                .d(0)
                .outputRange(-1, 1);
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
        leftIntakeMotor.set(speed);
    }

    public void stopIntakeMotor() {
        leftIntakeMotor.stopMotor();
    }

    public double getIntakeCurrent() {
        return leftIntakeMotor.getOutputCurrent();
    }

    public double getIntakeTemperature() {
        return leftIntakeMotor.getMotorTemperature();
    }

    public void pidSetPosition(PivotPositions position) {
        pivotMotorPID.setReference(position.getValue(), ControlType.kPosition);
    }
}