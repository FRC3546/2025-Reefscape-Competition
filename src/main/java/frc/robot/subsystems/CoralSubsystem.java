package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralSubsystem extends SubsystemBase {

    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    private SparkAbsoluteEncoder throughBoreEncoder;

    public CoralSubsystem() {

        pivotMotor = new SparkMax(71, MotorType.kBrushless);
        throughBoreEncoder = pivotMotor.getAbsoluteEncoder();
        intakeMotor = new SparkMax(72, MotorType.kBrushless);
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
}