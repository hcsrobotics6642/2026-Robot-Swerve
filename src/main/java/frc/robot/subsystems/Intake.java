package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Intake extends SubsystemBase {

    private final SparkMax m_FrontMotor = new SparkMax(constants.kFrontIntakeCanId, MotorType.kBrushless);
    private final SparkMax m_RearMotor = new SparkMax(constants.kRearIntakeCanId, MotorType.kBrushless);

    public Intake() {
        SparkMaxConfig config = new SparkMaxConfig();
        //config.smartCurrentLimit(40).idleMode(IdleMode.kCoast);

        m_FrontMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_RearMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setSpeed(double front, double rear) {
        m_FrontMotor.set(front);
        m_RearMotor.set(rear);
    }

    public void stop() {
        m_FrontMotor.set(0);
        m_RearMotor.set(0);
    }

    // These are the methods your RobotContainer is looking for
    public double getFrontCurrent() {
        return m_FrontMotor.getOutputCurrent();
    }

    public double getRearCurrent() {
        return m_RearMotor.getOutputCurrent();
    }
}