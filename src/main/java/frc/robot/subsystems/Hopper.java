package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// 2026 UPDATED: Top-level imports
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Hopper extends SubsystemBase {
    
    // The single motor used to eject/deploy the hopper
    // (Make sure to use whichever CAN ID you assigned to this motor in constants.java)
    private final SparkMax m_ejector = new SparkMax(constants.kHopper_ExpanderCanId, MotorType.kBrushless);

    public Hopper() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Brake mode is crucial so the hopper doesn't fall back in or flop around
        config.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
        
        // Basic PID loop so you can tell it exactly how far out to deploy
        config.closedLoop.p(0.1).i(0).d(0);

        m_ejector.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Commands the ejector to move to an exact encoder rotation.
     * Tune 'targetRotations' to find out how far it needs to spin to fully eject.
     */
    public void setPosition(double targetRotations) {
        m_ejector.getClosedLoopController().setSetpoint(
            targetRotations,
            SparkMax.ControlType.kPosition
        );
    }

    /** * Manual control of the ejector speed. 
     * Useful for binding to a joystick to safely test the deployment limit.
     */
    public void setSpeed(double speed) {
        m_ejector.set(speed);
    }

    /** Stops the ejector motor */
    public void stop() {
        m_ejector.stopMotor();
    }
}