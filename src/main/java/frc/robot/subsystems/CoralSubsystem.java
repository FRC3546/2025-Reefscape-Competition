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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class CoralSubsystem extends SubsystemBase {

    private SparkMax pivotMotor;
    private VictorSPX intakeMotor;
    private SparkAbsoluteEncoder throughBoreEncoder;

    private SparkClosedLoopController pivotMotorPID;
    private SparkMaxConfig pivotMotorConfig;

    public enum CoralPivotPositions {
        L1(0.2),
        L2(0),
        L3(0),
        L4(0);

        private final double value;

        CoralPivotPositions(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

    }

    public CoralSubsystem() {
        intakeMotor = new VictorSPX(52);
        pivotMotor = new SparkMax(51, MotorType.kBrushless);
        pivotMotorPID = pivotMotor.getClosedLoopController();
        pivotMotorConfig = new SparkMaxConfig();
        throughBoreEncoder = pivotMotor.getAbsoluteEncoder();

        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setInverted(true);

       
        pivotMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(4)
                .d(0)
                .outputRange(-.5, .5);
        pivotMotorConfig
            .inverted(true)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
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
        intakeMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void stopIntakeMotor() {
        intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public void pidSetPosition(CoralPivotPositions position) {
        pivotMotorPID.setReference(position.getValue(), ControlType.kPosition);
    }
}
