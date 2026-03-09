package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.generated.constants;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private final SparkMax m_motor =
            new SparkMax(constants.kTurretCanId, MotorType.kBrushless);

    private final CANcoder m_encoder =
            new CANcoder(constants.kTurretEncoderCanId);

    private final PIDController m_pid =
            new PIDController(constants.kTurretP,
                              constants.kTurretI,
                              constants.kTurretD);

    private double m_targetAngle = constants.kTurretCenterAngle;
    private boolean m_isClosedLoop = false;

    public Turret() {

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(25)
              .idleMode(IdleMode.kBrake);

        m_motor.configure(config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_pid.setTolerance(1.0);

        // AUTO CENTER ON BOOT
       // setAngle(constants.kTurretCenterAngle);
    }

    @Override
    public void periodic() {

        double currentAngle = getAngle();

        // Emergency hard stop
        if (currentAngle > constants.kTurretMaxAngle + 2 ||
            currentAngle < constants.kTurretMinAngle - 2) {

            m_motor.stopMotor();
            m_isClosedLoop = false;
            return;
        }

        if (m_isClosedLoop) {

            double output = m_pid.calculate(currentAngle, m_targetAngle);

            output = MathUtil.clamp(
                    output,
                    -constants.kTurretMaxOutput,
                     constants.kTurretMaxOutput);

            m_motor.set(output);
        }
    }

   public void setAngle(double degrees) {

    double normalized = normalizeAngle(degrees);

    if (!isWithinLimits(normalized)) {
        normalized = closestLimit(normalized);
    }

    m_targetAngle = normalized;
    m_isClosedLoop = true;
}

    public void setSpeed(double speed) {

        m_isClosedLoop = false;

        speed = MathUtil.clamp(
                speed,
                -constants.kTurretMaxOutput,
                 constants.kTurretMaxOutput);

        m_motor.set(speed);
    }

    public double getAngle() {

        return (m_encoder.getAbsolutePosition()
                .refresh()
                .getValueAsDouble() * 360.0)
                - constants.kTurretOffset;
    }

    public boolean isLocked(double targetAngle) {
        return Math.abs(getAngle() - targetAngle)
                < constants.kTurretLockToleranceDeg;
    }

    private double normalizeAngle(double angle) {
    angle %= 360.0;
    if (angle < 0) angle += 360.0;
    return angle;
}

private boolean isWithinLimits(double angle) {

    double min = constants.kTurretMinAngle;
    double max = constants.kTurretMaxAngle;

    // Wrapped range case
    if (min > max) {
        return angle >= min || angle <= max;
    }

    // Normal range
    return angle >= min && angle <= max;
}

private double closestLimit(double angle) {

    double min = constants.kTurretMinAngle;
    double max = constants.kTurretMaxAngle;

    double distToMin = angularDistance(angle, min);
    double distToMax = angularDistance(angle, max);

    return distToMin < distToMax ? min : max;
}

private double angularDistance(double a, double b) {
    double diff = Math.abs(a - b) % 360;
    return diff > 180 ? 360 - diff : diff;
}
    public void stop() {
        m_isClosedLoop = false;
        m_motor.stopMotor();
    }

    public void centerTurret() {
        setAngle(constants.kTurretCenterAngle);
    }
}
