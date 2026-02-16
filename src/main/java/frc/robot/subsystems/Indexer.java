package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    // Replace 34 with your actual CAN ID
    private final SparkMax m_motor = new SparkMax(34, MotorType.kBrushless);

    public Indexer() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // NEO 550s need lower current limits (20A)
        // Brake mode helps prevent fuel from coasting forward/backward
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);
        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Run at positive or negative percent output */
    public void setPercent(double speed) {
        m_motor.set(speed);
    }

    public void stop() {
        m_motor.stopMotor();
    }
}