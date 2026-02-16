package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Hood extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(constants.kHoodCanId, MotorType.kBrushless);
    private final CANcoder m_encoder = new CANcoder(constants.kHoodEncoderCanId);
    private final PIDController m_pid = new PIDController(0.05, 0, 0); // Tune these!

    // Map: Key = Distance (meters), Value = Hood Angle (degrees)
    private final InterpolatingDoubleTreeMap m_distanceMap = new InterpolatingDoubleTreeMap();

    public Hood() {
        // Example tuning: Distance vs Angle
        m_distanceMap.put(1.0, 15.0); // 1 meter away -> 15 degrees
        m_distanceMap.put(3.0, 35.0); // 3 meters away -> 35 degrees
        m_distanceMap.put(5.0, 45.0); // 5 meters away -> 45 degrees
    }

    public void setAngle(double angle) {
        double currentAngle = m_encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        double output = m_pid.calculate(currentAngle, angle);
        m_motor.set(output);
    }

    /**
     * Updates the hood based on Limelight distance
     */
    public void setAngleFromDistance(double distance) {
        double targetAngle = m_distanceMap.get(distance);
        setAngle(targetAngle);
    }

    /**
     * Checks if the hood is close enough to the target.
     * This fixes the error in SetHoodAngle.java
     */
    public boolean isAtPosition(double targetAngle, double tolerance) {
        double currentAngle = m_encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        return Math.abs(currentAngle - targetAngle) < tolerance;
    }

    public void stop() {
        m_motor.stopMotor();
    }
}