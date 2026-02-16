package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class FireFuel extends Command {
    private final Indexer m_indexer;
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final CommandXboxController m_controller;

    public FireFuel(Indexer indexer, Shooter shooter, Turret turret, CommandXboxController controller) {
        m_indexer = indexer;
        m_shooter = shooter;
        m_turret = turret;
        m_controller = controller;
        addRequirements(m_indexer);
    }

    @Override
    public void execute() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        boolean ready = m_shooter.isAtSpeed(100) && Math.abs(tx) < 2.0;

        if (ready) {
            m_indexer.setPercent(0.8);
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        } else {
            m_indexer.stop();
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_indexer.stop();
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }
}