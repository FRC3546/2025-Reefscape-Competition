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

public class CoralSubsystem extends SubsystemBase {

    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    private SparkAbsoluteEncoder throughBoreEncoder;

    private SparkClosedLoopController pivotMotorPID;
    private SparkMaxConfig pivotMotorConfig;

    public enum PivotPositions {
        L1(0),
        L2(0),
        L3(0),
        L4(0);

        private final double value;

        PivotPositions(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

    }

    public CoralSubsystem() {
        intakeMotor = new SparkMax(72, MotorType.kBrushless);
        pivotMotor = new SparkMax(71, MotorType.kBrushless);
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
        intakeMotor.set(speed);
    }

    public void stopIntakeMotor() {
        intakeMotor.stopMotor();
    }

    public double getIntakeCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    public double getIntakeTemperature() {
        return intakeMotor.getMotorTemperature();
    }

    public void pidSetPosition(PivotPositions position) {
        pivotMotorPID.setReference(position.getValue(), ControlType.kPosition);
    }
}