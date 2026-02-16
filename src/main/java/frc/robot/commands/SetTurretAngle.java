package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class SetTurretAngle extends Command {
    private final Turret m_turret;
    private final double m_angle;

    public SetTurretAngle(Turret turret, double angle) {
        m_turret = turret;
        m_angle = angle;
        addRequirements(m_turret);
    }

    @Override
    public void initialize() {
        m_turret.setAngle(m_angle);
    }

    @Override
    public boolean isFinished() {
        // Finishes when within 2 degrees of target
        return Math.abs(m_turret.getAngle() - m_angle) < 2.0;
    }
}