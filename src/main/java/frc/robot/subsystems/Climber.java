package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Climber extends SubsystemBase {
    // Hardware - Two TalonFXS controllers for two NEO 2.0 motors
    public final TalonFXS m_leftClimb = new TalonFXS(constants.kClimber_LeftCanId, "CC");
    public final TalonFXS m_rightClimb = new TalonFXS(constants.kClimber_RightCanId, "CC");

    private final PositionVoltage m_positionRequest = new PositionVoltage(0);

    public Climber() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();

        // 1. NEO 2.0 Integration
        CommutationConfigs commutation = new CommutationConfigs();
        commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
        config.Commutation = commutation;

        // 2. PID Gains for Position control - Adjust kP if the arm vibrates or is weak
        config.Slot0.kP = 2.0; 
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;

        // 3. Safety and Neutral Mode: Brake is required to prevent sliding down
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply config to both motors
        m_leftClimb.getConfigurator().apply(config);
        m_rightClimb.getConfigurator().apply(config);

        // 4. Set right motor to follow left motor but INVERTED
        // Since they face opposite directions on the climber, one must be inverted (true)
        m_rightClimb.setControl(new Follower(m_leftClimb.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setClimbPosition(double targetRotations) {
        m_leftClimb.setControl(m_positionRequest.withPosition(targetRotations));
    }

    public double getLeaderPosition() {
        // Essential for the Command to know when to stop
        return m_leftClimb.getPosition().refresh().getValueAsDouble();
    }

    public void stopMotor() {
        m_leftClimb.stopMotor();
        m_rightClimb.stopMotor();
    } 

    @Override
    public void periodic() {
        // Monitoring logic can go here
    }
}