package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

// 2026 UPDATED: Top-level imports
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Hopper extends SubsystemBase {
    private final SparkMax m_leftHopper = new SparkMax(constants.kHopper_LeftCanId, MotorType.kBrushless);
    private final SparkMax m_rightHopper = new SparkMax(constants.kHopper_RightCanId, MotorType.kBrushless);

    public Hopper() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.smartCurrentLimit(30);

        // Apply config using 2026 non-deprecated enums
        m_leftHopper.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        config.follow(m_leftHopper, true);
        m_rightHopper.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Moves the hopper to a specific distance.
     */
    public void setDistance(double inches) {
        double targetRotations = inches * 2.0; 
        // 2026 UPDATED: Use setSetpoint instead of setReference
        m_leftHopper.getClosedLoopController().setSetpoint(
            targetRotations, 
            SparkMax.ControlType.kPosition
        );
    }

    public boolean isAtPosition(double target, double tolerance) {
        return Math.abs(m_leftHopper.getEncoder().getPosition() - target) < tolerance;
    }

    public void runHopper(double speed) {
        m_leftHopper.set(speed);
    }

    public void stop() {
        m_leftHopper.stopMotor();
    }
}