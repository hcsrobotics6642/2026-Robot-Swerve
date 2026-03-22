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

    public Hood() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_pid.setTolerance(0.5);
    }

    @Override
    public void periodic() {
        // Closed-loop control always running toward current m_targetAngle
        double currentAngle = getAngle();
        double output = m_pid.calculate(currentAngle, m_targetAngle);
        m_motor.set(output);

        SmartDashboard.putNumber("Hood/Current Angle", currentAngle);
        SmartDashboard.putNumber("Hood/Target Angle", m_targetAngle);
    }

    public void setAngle(double degrees) {
        m_targetAngle = degrees;
    }

    public double getAngle() {
        // Refresh the encoder value and convert to degrees minus the mechanical offset
        return (m_encoder.getAbsolutePosition().refresh().getValueAsDouble() * 360.0) - constants.kHoodOffset;
    }

    public void stop() {
        m_motor.stopMotor();
    }
}
