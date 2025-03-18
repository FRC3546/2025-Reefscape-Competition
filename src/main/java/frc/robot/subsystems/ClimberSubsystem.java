package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberSubsystem extends SubsystemBase {

    private SparkMax leftClimbMotor;
    private SparkMaxConfig leftClimbMotorConfig;

    public ClimberSubsystem() {
        leftClimbMotor = new SparkMax(28, MotorType.kBrushless);

        leftClimbMotorConfig = new SparkMaxConfig();

        leftClimbMotorConfig
                .inverted(true)
                .smartCurrentLimit(40);

        leftClimbMotor.configure(leftClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setClimberSpeed(double speed) {
        leftClimbMotor.set(speed);
    }

    public void stopClimberMotor() {
        leftClimbMotor.stopMotor();
    }

    public double leftClimbMotorPosition() {
        return leftClimbMotor.getEncoder().getPosition();
    }
}