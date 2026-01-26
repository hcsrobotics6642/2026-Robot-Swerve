package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Hopper extends SubsystemBase {
    // Hardware - Two TalonFXS controllers for two NEO 2.0 motors
    private final TalonFXS m_leftMotorHopper = new TalonFXS(TunerConstants.kHopper_LeftCanId);
    private final TalonFXS m_rightMotorHopper = new TalonFXS(TunerConstants.kHopper_RightCanId);

    // Control Requests
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private final PositionVoltage m_positionRequest = new PositionVoltage(0);

    // Physical Constants
    // 60:1 Gearbox and 3 inches per output revolution
    // Formula: 60 motor revs / 3 inches = 20 motor revs per inch
    private final double kRotationsPerInch = 20.0;

    public Hopper() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();

        // 1. NEO 2.0 Integration
        CommutationConfigs commutation = new CommutationConfigs();
        commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
        config.Commutation = commutation;

        // 2. PID Gains
        // Slot 0: Velocity Control (RPM)
        config.Slot0.kP = 0.11; 
        config.Slot0.kV = 0.12; 

        // Slot 1: Position Control (Inches)
        // Note: Position control usually requires a higher P gain than velocity
        config.Slot1.kP = 2.5;
        config.Slot1.kD = 0.1;

        // 3. Safety and Neutral Mode
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Changed to Brake for precision

        // Apply config to both motors
        m_leftMotorHopper.getConfigurator().apply(config);
        m_rightMotorHopper.getConfigurator().apply(config);

        // 4. Follower setup
        m_rightMotorHopper.setControl(new Follower(m_leftMotorHopper.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Set the hopper speed in RPM.
     * @param targetRPM Desired Rotations Per Minute
     */
    public void setRPM(double targetRPM) {
        m_leftMotorHopper.setControl(m_velocityRequest.withVelocity(targetRPM / 60.0).withSlot(0));
    }

    /**
     * Move the hopper a specific distance in inches.
     * @param targetInches The absolute position in inches
     */
    public void setDistance(double targetInches) {
        double targetRotations = targetInches * kRotationsPerInch;
        m_leftMotorHopper.setControl(m_positionRequest.withPosition(targetRotations).withSlot(1));
    }
    public boolean isAtPosition(double targetInches, double toleranceInches) {
    return Math.abs(getPositionInches() - targetInches) < toleranceInches;
}
    /**
     * @return Current position in inches based on motor rotations
     */
    public double getPositionInches() {
        return m_leftMotorHopper.getPosition().getValueAsDouble() / kRotationsPerInch;
    }

    /**
     * Resets the encoder to zero inches.
     */
    public void resetEncoder() {
        m_leftMotorHopper.setPosition(0);
    }

    public void stop() {
        m_leftMotorHopper.setControl(m_velocityRequest.withVelocity(0));
    }

    @Override
    public void periodic() {
        // Telemetry or Limelight logic
    }
}