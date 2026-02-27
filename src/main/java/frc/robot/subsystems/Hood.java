package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Hood extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(constants.kHoodCanId, MotorType.kBrushless);
    private final CANcoder m_encoder = new CANcoder(constants.kHoodEncoderCanId);

    private final PIDController m_pid = new PIDController(0.1, 0, 0); 
    private double m_targetAngle = 0;
    private boolean m_isClosedLoop = false;

    public Hood() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);
        
        // Applying 2026 configuration
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_pid.setTolerance(0.5); 
    }
    

    @Override
    public void periodic() {
        double currentAngle = getAngle();

        if (m_isClosedLoop) {
            double output = m_pid.calculate(currentAngle, m_targetAngle);
            m_motor.set(output);
        }

        // Useful for finding that offset later!
        SmartDashboard.putNumber("Hood/Angle", currentAngle);
        SmartDashboard.putNumber("Hood/Raw Degrees", m_encoder.getAbsolutePosition().getValueAsDouble() * 360.0);
    }

    public boolean isAtPosition(double target, double tolerance) {
        return Math.abs(getAngle() - target) < tolerance;
    }
    
    public void setAngleFromDistance(double distance) {
        // formula: (distance * multiplier) + vertical_offset
        // You will tune these numbers during practice!
        double targetAngle = (distance * 1.5) + 10.0; 
        setAngle(targetAngle);
    }
    
    public void setAngle(double degrees) {
        m_targetAngle = degrees;
        m_isClosedLoop = true;
    }

    public double getAngle() {
        // Rotations to Degrees minus the constant offset
        return (m_encoder.getAbsolutePosition().refresh().getValueAsDouble() * 360.0) - constants.kHoodOffset;
    }

    public void stop() {
        m_isClosedLoop = false;
        m_motor.stopMotor();
    }
}