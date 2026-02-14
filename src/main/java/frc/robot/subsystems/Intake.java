package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Intake extends SubsystemBase {
    // Hardware - Using SparkMax for NEO/NEO 550 motors
    private final SparkMax m_FrontMotor = new SparkMax(constants.kFrontIntakeCanId, MotorType.kBrushless);
    private final SparkMax m_RearMotor = new SparkMax(constants.kRearIntakeCanId, MotorType.kBrushless);
    
    public Intake() {
        /*
         * Create a configuration object for the Spark Max.
         * In REVLib 2025+, this is the preferred way to set limits and modes.
         */
        SparkMaxConfig config = new SparkMaxConfig();

        // Current limits to protect the NEO motors
        config.smartCurrentLimit(40);

        // Idle mode - kCoast allows the intake to spin down naturally
        config.idleMode(IdleMode.kCoast);

        /* * Apply the configuration to the motors.
         * kResetSafeParameters ensures a clean state, kPersistParameters saves to flash.
         */
        m_FrontMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_RearMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Set motor power using percentage.
     * @param speed from -1.0 to 1.0
     * @param Rspeed from -1.0 to 1.0
     */
    public void setSpeed(double speed, double Rspeed) {
        m_FrontMotor.set(speed);
        m_RearMotor.set(Rspeed);
    }

    /**
     * Stops the motors immediately.
     */
    public void stop() {
        m_FrontMotor.stopMotor();
        m_RearMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // Runs every 20ms
    }
}