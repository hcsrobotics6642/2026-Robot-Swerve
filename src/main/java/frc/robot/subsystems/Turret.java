package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.generated.constants;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    // Hardware - IDs should be checked in your Phoenix Tuner / REV Hardware Client
    private final SparkMax m_motor = new SparkMax(constants.kTurretCanId , MotorType.kBrushless);
    private final CANcoder m_encoder = new CANcoder(constants.kTurretEncoderCanId ); // roboRIO Bus (no "CC")

    // PID Controller: kP 0.05 is a safe starting point for a NEO 550
    private final PIDController m_pid = new PIDController(0.05, 0, 0);

    private double m_targetAngle = 0;
    private boolean m_isClosedLoop = false;

    public Turret() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Safety for NEO 550
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);
        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Define tolerance so the PID knows when it is "close enough"
        m_pid.setTolerance(1.0); // 1 degree
    }

    @Override
    public void periodic() {
        // If we have set an angle, the RIO calculates the power needed and sends it to Spark Max
        if (m_isClosedLoop) {
            double output = m_pid.calculate(getAngle(), m_targetAngle);
            m_motor.set(output);
        }
    }

    public void setAngle(double degrees) {
        m_targetAngle = degrees;
        m_isClosedLoop = true;
    }

    public void setSpeed(double speed) {
        m_isClosedLoop = false;
        m_motor.set(speed);
    }

    /**
     * WCP/CTRE CANcoder returns rotations. 
     * getValueAsDouble() * 360 gives degrees.
     */
    public double getAngle() {
        return m_encoder.getAbsolutePosition().refresh().getValueAsDouble() * 360.0;
    }

    public boolean isAtPosition(double target, double tolerance) {
        return Math.abs(getAngle() - target) < tolerance;
    }

    public void stop() {
        m_isClosedLoop = false;
        m_motor.stopMotor();
    }
}