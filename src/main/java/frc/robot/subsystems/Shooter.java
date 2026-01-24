package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut; // Use Voltage for consistency
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Shooter extends SubsystemBase {
    private final TalonFX Shooter_motor = new TalonFX(TunerConstants.kShooter_LeftCanId);
    private final VoltageOut m_voltageControl = new VoltageOut(0);

    public Shooter(Shooter m_shooter, double d) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // 1. Set Current Limits (Supply = 30A, Stator = 40A)
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // 2. Set Neutral Mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast is often smoother for intakes

        Shooter_motor.getConfigurator().apply(config);
    }

    /**
     * @param volts Desired voltage (typically 10-12V for full power)
     */
    public void setIntakeVoltage(double volts) {
       Shooter_motor.setControl(m_voltageControl.withOutput(volts));
    }

    public void stop() {
        Shooter_motor.setControl(m_voltageControl.withOutput(0));
    }

    // Example of "sensing" a game piece
    public boolean isJammed() {
        return Shooter_motor.getStatorCurrent().getValueAsDouble() > 35.0;
    }

    public void setMotorSpeed(double m_speed) {
      // TODO Auto-generated method stub
      Shooter_motor.setControl(m_voltageControl.withOutput(0.9));
    }
}