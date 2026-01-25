package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Shooter extends SubsystemBase {
    // Hardware - Two TalonFXS controllers for two NEO 2.0 motors
    private final TalonFXS m_leftMotor = new TalonFXS(TunerConstants.kShooter_LeftCanId);
    private final TalonFXS m_rightMotor = new TalonFXS(TunerConstants.kShooter_RightCanId);

    // Control Request for precise RPM
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    public Shooter() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();

        // 1. NEO 2.0 Integration
        CommutationConfigs commutation = new CommutationConfigs();
        commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
        config.Commutation = commutation;

        // 2. PID Gains for RPM control
        config.Slot0.kP = 0.11; 
        config.Slot0.kV = 0.12; 

        // 3. Safety and Neutral Mode
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Apply config to both motors
        m_leftMotor.getConfigurator().apply(config);
        m_rightMotor.getConfigurator().apply(config);

        // 4. Set right motor to follow left motor with opposite alignment
        // MotorAlignmentValue.Opposed means the follower spins opposite to the leader
        // This is correct for two motors on opposite sides of a flywheel
        m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * @param targetRPM Desired Rotations Per Minute
     */
    public void setRPM(double targetRPM) {
        // RPS = RPM / 60
        // Only control the left motor - right motor follows automatically
        m_leftMotor.setControl(m_velocityRequest.withVelocity(targetRPM / 60.0));
    }

    public void stop() {
        m_leftMotor.setControl(m_velocityRequest.withVelocity(0));
    }

    @Override
    public void periodic() {
        // Limelight distance logic will go here
    }
}