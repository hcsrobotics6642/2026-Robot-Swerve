package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

public class TurretTrackTarget extends Command {

    private final Turret m_turret;
    private final DoubleSupplier m_angleSupplier;

    public TurretTrackTarget(
            Turret turret,
            DoubleSupplier angleSupplier) {

        m_turret = turret;
        m_angleSupplier = angleSupplier;

        addRequirements(m_turret);
    }

    @Override
    public void execute() {

        double tv = NetworkTableInstance.getDefault()
                .getTable("limelight_tauret")
                .getEntry("tv")
                .getDouble(0.0);

        if (tv == 1.0) {
            double targetAngle = m_angleSupplier.getAsDouble();
            m_turret.setAngle(targetAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}