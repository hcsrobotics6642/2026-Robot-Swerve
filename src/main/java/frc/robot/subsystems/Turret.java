package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.generated.constants;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Added for debugging
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(constants.kTurretCanId, MotorType.kBrushless);
    private final CANcoder m_encoder = new CANcoder(constants.kTurretEncoderCanId);
    private final PIDController m_pid = new PIDController(0.05, 0, 0);

    private double m_targetAngle = 0;
    private boolean m_isClosedLoop = false;

    public Turret() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(25).idleMode(IdleMode.kBrake);

        // Hardware Soft Limits
        config.softLimit
            .forwardSoftLimit(constants.kTurretMaxAngle)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(constants.kTurretMinAngle)
            .reverseSoftLimitEnabled(true);
        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_pid.setTolerance(1.0);

        Shuffleboard.getTab("Tuning").add("Turret PID", m_pid);
    }

    @Override
    public void periodic() {
        double currentAngle = getAngle();
        
        if (m_isClosedLoop) {
            double output = m_pid.calculate(currentAngle, m_targetAngle);
            m_motor.set(output);
        }

        // Helpful for finding your kTurretOffset!
        SmartDashboard.putNumber("Turret/Current Angle", currentAngle);
        SmartDashboard.putNumber("Turret/Target Angle", m_targetAngle);
        SmartDashboard.putNumber("Turret/Raw CANcoder Degrees", m_encoder.getAbsolutePosition().getValueAsDouble() * 360.0);
    }

    public void setAngle(double degrees) {
        m_targetAngle = MathUtil.clamp(degrees, constants.kTurretMinAngle, constants.kTurretMaxAngle);
        m_isClosedLoop = true;
    }

    public void setSpeed(double speed) {
        m_isClosedLoop = false; // Turn off PID so it doesn't fight the manual speed
        m_motor.set(speed);
    }

    public double getAngle() {
        // Rotations to Degrees minus our calibration offset
        return (m_encoder.getAbsolutePosition().refresh().getValueAsDouble() * 360.0) - constants.kTurretOffset;
    }

    public boolean isAtPosition(double target, double tolerance) {
        return Math.abs(getAngle() - target) < tolerance;
    }

    public void stop() {
        m_isClosedLoop = false;
        m_motor.stopMotor();
    }
}