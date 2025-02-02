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

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax leftElevatorMotor;
    private SparkMax rightElevatorMotor;
    private SparkAbsoluteEncoder throughBoreEncoder;
    private SparkClosedLoopController leftElevatorMotorPID;
    private SparkMaxConfig leftElevatorMotorConfig;
    private SparkMaxConfig rightElevatorMotorConfig;

    public enum ElevatorPositions {
        L1(0),
        L2(0),
        L3(0),
        L4(0),
        Barge(0),
        AlgaeReefHigh(0),
        AlgaeReefLow(0),
        Processor(0);

        private final double value;

        ElevatorPositions(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

    }

    public ElevatorSubsystem() {
        leftElevatorMotor = new SparkMax(0, MotorType.kBrushless);
        rightElevatorMotor = new SparkMax(0, MotorType.kBrushless);
        leftElevatorMotorPID = leftElevatorMotor.getClosedLoopController();
        leftElevatorMotorConfig = new SparkMaxConfig();
        rightElevatorMotorConfig = new SparkMaxConfig();
        throughBoreEncoder = leftElevatorMotor.getAbsoluteEncoder();

        leftElevatorMotorConfig.closedLoop
                .p(0.001)
                .d(0)
                .outputRange(-1, 1);
        leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // right motor is follower
        rightElevatorMotorConfig.follow(leftElevatorMotor.getDeviceId(), false);
        rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public double getElevatorPosition() {
        return throughBoreEncoder.getPosition();
    }

    public double getElevatorVelocity() {
        return throughBoreEncoder.getVelocity();
    }

    public void setElevatorSpeed(double speed){
        leftElevatorMotor.set(speed);
    }

    public void pidSetPosition(ElevatorPositions position) {
        leftElevatorMotorPID.setReference(position.getValue(), ControlType.kPosition);
    }
}