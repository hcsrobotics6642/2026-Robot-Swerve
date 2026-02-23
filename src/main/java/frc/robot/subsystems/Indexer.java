package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// 2026 UPDATE: Use the top-level com.revrobotics imports
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import frc.robot.generated.constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    // Replace the number 34 with the constant name
    private final SparkMax m_motor = new SparkMax(constants.kIndexerCanId, MotorType.kBrushless);

    public Indexer() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // NEO 550s need lower current limits (20A)
        // Brake mode helps prevent fuel from coasting forward/backward
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);
        
        // 2026 UPDATE: Using top-level ResetMode and PersistMode
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