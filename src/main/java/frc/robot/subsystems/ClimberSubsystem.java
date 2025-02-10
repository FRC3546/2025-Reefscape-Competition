package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberSubsystem extends SubsystemBase {

    private SparkMax backClimbMotor;
    private SparkMax frontClimbMotor;
    private SparkMaxConfig backClimbMotorConfig;
    private SparkMaxConfig frontClimbMotorConfig;

    public ClimberSubsystem() {
        backClimbMotor = new SparkMax(52, MotorType.kBrushless);
        frontClimbMotor = new SparkMax(53, MotorType.kBrushless);

        backClimbMotorConfig = new SparkMaxConfig();
        frontClimbMotorConfig = new SparkMaxConfig();

        backClimbMotorConfig
                .inverted(false)
                .smartCurrentLimit(40);
        frontClimbMotorConfig
                .inverted(false)
                .smartCurrentLimit(40)
                .follow(backClimbMotor.getDeviceId());

        backClimbMotor.configure(backClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frontClimbMotor.configure(frontClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setClimberSpeed(double speed){
        backClimbMotor.set(speed);
    }

    public void stopClimberMotor(){
        backClimbMotor.stopMotor();
    }
}