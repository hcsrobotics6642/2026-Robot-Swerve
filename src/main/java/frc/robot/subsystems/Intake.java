package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Intake extends SubsystemBase {
    // Hardware
    // Change this line in Intake.java
private final TalonFX m_motor = new TalonFX(TunerConstants.kIntakeCanId, "");
    
    // Control Request (pre-allocated for performance)
    private final VoltageOut m_voltageRequest = new VoltageOut(10);

    public Intake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Apply configuration and check for errors
    var status = m_motor.getConfigurator().apply(config);
    if (!status.isOK()) {
        System.err.println("Failed to configure intake motor: " + status.toString());
    }

        

        // Current limits prevent the motor from burning out if jammed
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Neutral mode: Coast is usually better for intakes to avoid bouncing game pieces
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Apply configuration to the motor
        m_motor.getConfigurator().apply(config);
    }

    /**
     * Set motor power using percentage.
     * @param speed from -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        // Convert -1.0..1.0 range to -12V..12V range
        System.out.println("Intake speed set to: " + speed);
        m_motor.setControl(m_voltageRequest.withOutput(speed * 12.0));
    }

    /**
     * Stops the motor immediately.
     */
    public void stop() {
        m_motor.setControl(m_voltageRequest.withOutput(0));
    }

    @Override
    public void periodic() {
        // This runs 50 times per second. 
        // You can add SmartDashboard telemetry here later.
    }
}