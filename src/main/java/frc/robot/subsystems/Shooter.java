package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage; // For RPM control
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue; // For inversion logic
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Shooter extends SubsystemBase {
    private final TalonFX m_leftMotor = new TalonFX(constants.kShooter_LeftCanId);
    private final TalonFX m_rightMotor = new TalonFX(constants.kShooter_RightCanId);
    
    // Velocity request object - Phoenix 6 uses this for closed-loop control
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private double m_targetRPM = 0;

    public Shooter() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Safety & Physics
        config.CurrentLimits.StatorCurrentLimit = 80.0; 
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // PID Gains (Slot 0) - These are starting points
        config.Slot0.kP = constants.kShooter_kP;
        config.Slot0.kV = constants.kShooter_kV;

        // Apply config to the leader motor
        m_leftMotor.getConfigurator().apply(config);
        
        // Right motor follows left, but spins in the OPPOSITE direction 
        // to propel the game piece forward.
        m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Set the target speed of the shooter.
     * @param targetRPM The desired speed in Revolutions Per Minute.
     */
    public void setRPM(double targetRPM) {
        m_targetRPM = targetRPM;
        // Phoenix 6 expects Rotations Per Second (RPS)
        double targetRPS = targetRPM / 60.0;
        m_leftMotor.setControl(m_velocityRequest.withVelocity(targetRPS));
    }

    public double getCurrentRPM() {
        // Refresh the signal and convert RPS to RPM
        return m_leftMotor.getVelocity().refresh().getValueAsDouble() * 60.0;
    }

    public boolean isAtSpeed(double toleranceRPM) {
        return Math.abs(getCurrentRPM() - m_targetRPM) < toleranceRPM;
    }

    public void stop() {
        m_targetRPM = 0;
        m_leftMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Current RPM", getCurrentRPM());
        SmartDashboard.putNumber("Shooter/Target RPM", m_targetRPM);
        SmartDashboard.putBoolean("Shooter/At Speed", isAtSpeed(150));
    }
}