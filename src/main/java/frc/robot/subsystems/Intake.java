package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue; // Required for NEO 2.0 support
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Intake extends SubsystemBase {
    // Hardware - Using TalonFXS class for the Talon FX S controller
    private final TalonFXS m_FrontMotor = new TalonFXS(TunerConstants.kFrontIntakeCanId, "");
    private final TalonFXS m_RearMotor = new TalonFXS(TunerConstants.kRearIntakeCanId, "");
    // DutyCycleOut handles percentage-based power (-1.0 to 1.0)
    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

    public Intake() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();

       CommutationConfigs commutation = new CommutationConfigs();
        commutation.MotorArrangement = MotorArrangementValue.NEO_JST; // Specific to REV NEO motors
        config.Commutation = commutation;

        // Current limits to protect the NEO 2.0 motor from overheating
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Neutral mode - Coast allows the intake to spin down naturally
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Apply configuration and print status to the console for debugging
        m_FrontMotor.getConfigurator().apply(config);
        m_RearMotor.getConfigurator().apply(config);
       
    }

    /**
     * Set motor power using percentage.
     * @param speed from -1.0 to 1.0 (e.g., 0.9 for 90% power)
     */
    public void setSpeed(double speed, double Rspeed) {
        // Fixed: Now uses the 'speed' parameter instead of hardcoded 12
        m_FrontMotor.setControl(m_dutyCycleRequest.withOutput(speed));
        m_RearMotor.setControl(m_dutyCycleRequest.withOutput(Rspeed));
    }

    /**
     * Stops the motor immediately.
     */
    public void stop() {
        m_FrontMotor.setControl(m_dutyCycleRequest.withOutput(0));
         m_RearMotor.setControl(m_dutyCycleRequest.withOutput(0));
    }

    @Override
    public void periodic() {
        // Runs every 20ms
    }
}