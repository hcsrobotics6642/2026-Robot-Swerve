package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Shooter extends SubsystemBase {
    private final TalonFX m_leftMotor = new TalonFX(constants.kShooter_LeftCanId);
    private final TalonFX m_rightMotor = new TalonFX(constants.kShooter_RightCanId);
    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
    private double m_targetRPM = 0;

    public Shooter() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        m_leftMotor.getConfigurator().apply(config);
        m_rightMotor.getConfigurator().apply(config);

        // Right motor follows left but spins the opposite way for a shooter
        m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    
   public void setRPM(double targetRPM) {
    m_targetRPM = targetRPM;
    // Convert RPM to Rotations per Second for Phoenix 6
    double rps = targetRPM / 60.0; 
    m_leftMotor.setControl(m_velocityRequest.withVelocity(rps));
    }

    public boolean isAtSpeed(double toleranceRPM) {
        double currentRPM = Math.abs(m_leftMotor.getVelocity().getValueAsDouble() * 60.0);
        return Math.abs(currentRPM - Math.abs(m_targetRPM)) < toleranceRPM;
    }

    public void stop() {
        m_targetRPM = 0;
        m_leftMotor.setControl(m_dutyCycleRequest.withOutput(0));
    }

   // Inside Shooter.java
@Override
public void periodic() {
    // Current draws help you see if a motor is straining
    double leftCurrent = m_leftMotor.getStatorCurrent().getValueAsDouble();
    double rightCurrent = m_rightMotor.getStatorCurrent().getValueAsDouble();
    
    // Temperatures let you know if you're geared too high
    double leftTemp = m_leftMotor.getDeviceTemp().getValueAsDouble();
    
    SmartDashboard.putNumber("Diagnostics/Shooter Temp (C)", leftTemp);
    SmartDashboard.putNumber("Diagnostics/Shooter Amps", leftCurrent);

    // Alert the driver if the motors are getting dangerously hot (over 70C)
    if (leftTemp > 70.0) {
        SmartDashboard.putBoolean("Diagnostics/SHOOTER OVERHEATING", true);
    } else {
        SmartDashboard.putBoolean("Diagnostics/SHOOTER OVERHEATING", false);
    }
}
}