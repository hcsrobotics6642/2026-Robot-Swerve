package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Turret extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(constants.kTurretCanId, MotorType.kBrushless);
    private final CANcoder m_encoder = new CANcoder(constants.kTurretEncoderCanId);

    private final PIDController m_pid = new PIDController(0.05, 0, 0);
    private double m_targetAngle = 0;
    private boolean m_isClosedLoop = false;
    
    public Turret() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);

        // 2026 UPDATE: Enable Soft Limits at the hardware level
        // Note: This requires the motor's internal encoder to be synced with the CANcoder
        config.softLimit.forwardSoftLimit(constants.kTurretMaxAngle)
                         .forwardSoftLimitEnabled(true)
                         .reverseSoftLimit(constants.kTurretMinAngle)
                         .reverseSoftLimitEnabled(true);
        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_pid.setTolerance(1.0);
    }

    public void setAngle(double degrees) {
        // Clamp the input so the PID controller never tries to reach an impossible goal
        m_targetAngle = MathUtil.clamp(degrees, constants.kTurretMinAngle, constants.kTurretMaxAngle);
        m_isClosedLoop = true;
    }

    @Override
    public void periodic() {
        if (m_isClosedLoop) {
            double currentAngle = getAngle();
            double output = m_pid.calculate(currentAngle, m_targetAngle);

            // Safety speed clamp (don't go faster than 25% power)
            double maxSpeed = 0.25;
            output = Math.copySign(Math.min(Math.abs(output), maxSpeed), output);

            m_motor.set(output);
        }
        SmartDashboard.putNumber("Turret/Angle", getAngle());
    }

    public void setSpeed(double speed) {
        m_isClosedLoop = false;
        m_motor.set(speed);
    }

    public double getAngle() {
        // Refresh ensures we get the latest data from the CAN bus
        return m_encoder.getAbsolutePosition().refresh().getValueAsDouble() * 360.0;
    }

    public void stop() {
        m_isClosedLoop = false;
        m_motor.set(0);
    }
}