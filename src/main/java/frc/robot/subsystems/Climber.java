package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// 2026 UPDATED: Top-level imports for Persist and Reset modes
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Climber extends SubsystemBase {
    private final SparkMax m_leftClimb = new SparkMax(constants.kClimber_LeftCanId, MotorType.kBrushless);
    private final SparkMax m_rightClimb = new SparkMax(constants.kClimber_RightCanId, MotorType.kBrushless);

    public Climber() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Use Brake mode for climbers to prevent the robot from falling
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(50);

        // Setup PID for position control
        config.closedLoop.p(2.0).i(0.0).d(0.1);

        // Apply configuration using 2026 non-deprecated enums
        m_leftClimb.configure(
            config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        // Set right to follow left inverted
        config.follow(m_leftClimb, true);
        m_rightClimb.configure(
            config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
    }

    /**
     * UPDATED: Uses the non-deprecated setSetpoint method for 2026.
     * This replaces the deprecated setReference method.
     */
    public void setClimbPosition(double targetRotations) {
        m_leftClimb.getClosedLoopController().setSetpoint(
            targetRotations, 
            SparkMax.ControlType.kPosition
        );
    }

    public double getLeaderPosition() {
        return m_leftClimb.getEncoder().getPosition();
    }

    public void stopMotor() {
        m_leftClimb.stopMotor();
    }
}