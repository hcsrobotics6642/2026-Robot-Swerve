package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// 2026 UPDATE: Direct imports from com.revrobotics
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.generated.constants;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(constants.kTurretCanId , MotorType.kBrushless);
    private final CANcoder m_encoder = new CANcoder(constants.kTurretEncoderCanId );

    private final PIDController m_pid = new PIDController(0.05, 0, 0);

    private double m_targetAngle = 0;
    private boolean m_isClosedLoop = false;

    public Turret() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);
        
        // 2026 UPDATE: Apply configuration using top-level enums
        m_motor.configure(
            config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        m_pid.setTolerance(1.0);
    }

    @Override
    public void periodic() {
        if (m_isClosedLoop) {
            double output = m_pid.calculate(getAngle(), m_targetAngle);
            m_motor.set(output);
        }
    }

    public void setAngle(double degrees) {
        m_targetAngle = degrees;
        m_isClosedLoop = true;
    }

    public void setSpeed(double speed) {
        m_isClosedLoop = false;
        m_motor.set(speed);
    }

    public double getAngle() {
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