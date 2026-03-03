package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.generated.constants;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    // 1. Create a dedicated Tab for this subsystem
    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Turret");
    
    private final SparkMax m_motor = new SparkMax(constants.kTurretCanId, MotorType.kBrushless);
    private final CANcoder m_encoder = new CANcoder(constants.kTurretEncoderCanId);
    private final PIDController m_pid = new PIDController(0.05, 0, 0);

    private double m_targetAngle = 0;
    private boolean m_isClosedLoop = false;

    public Turret() {
        // 2. Add the PID Controller to the dashboard so we can tune P, I, and D live
        m_tab.add("Turret PID", m_pid);
        
        // Also add it to a general "Tuning" tab for convenience
        Shuffleboard.getTab("Tuning").add("Turret PID Object", m_pid);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(25).idleMode(IdleMode.kBrake);

        /* * NOTE: If kTurretMaxAngle is red, you MUST add it to constants.java. 
         * For now, I've left these active assuming you will add them next.
         */
        config.softLimit
            .forwardSoftLimit(constants.kTurretMaxAngle)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(constants.kTurretMinAngle)
            .reverseSoftLimitEnabled(true);
        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_pid.setTolerance(1.0);
    }

    @Override
    public void periodic() {
        double currentAngle = getAngle();
        
        if (m_isClosedLoop) {
            double output = m_pid.calculate(currentAngle, m_targetAngle);
            m_motor.set(output);
        }

        // 3. Telemetry: Send data to the dashboard every loop
        SmartDashboard.putNumber("Turret/Current Angle", currentAngle);
        SmartDashboard.putNumber("Turret/Target Angle", m_targetAngle);
        
        // Display raw sensor data to help calibrate your constants.kTurretOffset
        SmartDashboard.putNumber("Turret/Raw Encoder", m_encoder.getAbsolutePosition().getValueAsDouble() * 360.0);
    }

    public void setAngle(double degrees) {
        m_targetAngle = MathUtil.clamp(degrees, constants.kTurretMinAngle, constants.kTurretMaxAngle);
        m_isClosedLoop = true;
    }

    public void setSpeed(double speed) {
        m_isClosedLoop = false; 
        m_motor.set(speed);
    }

    public double getAngle() {
        /* * 2026 UPDATE: .refresh() is critical for Phoenix 6. 
         * It ensures we aren't using "stale" data from the previous loop.
         */
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