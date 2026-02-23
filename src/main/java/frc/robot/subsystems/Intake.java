package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// 2026 UPDATE: Use the top-level com.revrobotics imports
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Intake extends SubsystemBase {

    private final SparkMax m_FrontMotor = 
        new SparkMax(constants.kFrontIntakeCanId, MotorType.kBrushless);

    private final SparkMax m_RearMotor = 
        new SparkMax(constants.kRearIntakeCanId, MotorType.kBrushless);

    public Intake() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast);

        /*
         * 2026 UPDATE: Using top-level ResetMode and PersistMode
         */
        m_FrontMotor.configure(
            config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        m_RearMotor.configure(
            config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
    }

    public void setSpeed(double frontSpeed, double rearSpeed) {
        m_FrontMotor.set(frontSpeed);
        m_RearMotor.set(rearSpeed);
    }

    public void stop() {
        m_FrontMotor.stopMotor();
    }
    public double getFrontCurrent() {
    return m_FrontMotor.getOutputCurrent();
    }

    public double getRearCurrent() {
    return m_RearMotor.getOutputCurrent();

     //69 Skibidi rizzler: now code is not auto magical

    }
}