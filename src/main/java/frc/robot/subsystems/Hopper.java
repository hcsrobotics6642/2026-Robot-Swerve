package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Hopper extends SubsystemBase {
    // Hardware - Two Spark Maxes for the Hopper
    // No bus name specified, so they default to the roboRIO CAN bus
    private final SparkMax m_leftHopper = new SparkMax(constants.kHopper_LeftCanId, MotorType.kBrushless);
    private final SparkMax m_rightHopper = new SparkMax(constants.kHopper_RightCanId, MotorType.kBrushless);

    public Hopper() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Basic configuration
        config.smartCurrentLimit(30);

        // Apply config to left motor
        m_leftHopper.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Configure right motor to follow the left motor
        // Set 'true' or 'false' for the second parameter depending on your physical mounting
        config.follow(m_leftHopper, true);
        m_rightHopper.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Moves the hopper to a specific distance.
     * @param inches The distance to move.
     */
    public void setDistance(double inches) {
        // Convert inches to motor rotations. 
        // Example: if 1 inch = 2 motor rotations, use (inches * 2.0)
        double targetRotations = inches * 2.0; 
        m_leftHopper.getClosedLoopController().setReference(targetRotations, SparkMax.ControlType.kPosition);
    }

    /**
     * Checks if the hopper has reached its target position.
     * This fixes the red error in HopperOut.java.
     * @param target The target position (rotations/inches).
     * @param tolerance The allowable error (e.g., 0.5).
     * @return true if the current position is within the tolerance of the target.
     */
    public boolean isAtPosition(double target, double tolerance) {
        return Math.abs(m_leftHopper.getEncoder().getPosition() - target) < tolerance;
    }

    /**
     * Manually run the hopper at a specific speed.
     */
    public void runHopper(double speed) {
        m_leftHopper.set(speed);
    }

    /**
     * Stops the hopper motors.
     */
    public void stop() {
        m_leftHopper.stopMotor();
    }
}