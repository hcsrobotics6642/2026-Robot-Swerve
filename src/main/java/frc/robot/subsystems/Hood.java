package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Hood extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(constants.kHoodCanId, MotorType.kBrushless);
    private final CANcoder m_encoder = new CANcoder(constants.kHoodEncoderCanId);

    // 2026 UPDATE: Software PID for external encoder
    private final PIDController m_pid = new PIDController(0.1, 0, 0); 
    private double m_targetAngle = 0;
    private boolean m_isClosedLoop = false;

    public Hood() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);
        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_pid.setTolerance(0.5); // 0.5 degree tolerance
    }

    @Override
    public void periodic() {
        if (m_isClosedLoop) {
            // Calculate RIO-side PID output
            double output = m_pid.calculate(getAngle(), m_targetAngle);
            m_motor.set(output);
        }
    }

    public void setAngle(double degrees) {
        m_targetAngle = degrees;
        m_isClosedLoop = true;
    }

    public void setAngleFromDistance(double distance) {
        // Logic to map distance to a specific angle (linear interpolation or equation)
        double targetAngle = (distance * 1.5) + 10; // Example placeholder logic
        setAngle(targetAngle);
    }

    public double getAngle() {
        // getValueAsDouble() returns rotations; * 360 converts to degrees
        return m_encoder.getAbsolutePosition().refresh().getValueAsDouble() * 360.0;
    }

    public boolean isAtPosition(double target, double tolerance) {
        return Math.abs(getAngle() - target) < tolerance;
    }

    public void stop() {
        m_isClosedLoop = false;
        m_motor.stopMotor();
    }
}