package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Intake;

public class FireFuel extends Command {
    private final Indexer m_indexer;
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Intake m_intake;
    private final CommandXboxController m_controller;

    public FireFuel( Intake intake, Indexer indexer, Shooter shooter, Turret turret, CommandXboxController controller) {
        m_indexer = indexer;
        m_shooter = shooter;
        m_turret = turret;
        m_controller = controller;
        m_intake = intake;
        addRequirements(m_indexer);
    }

    @Override
    public void execute() {
        // 1. Get Limelight data
        var table = NetworkTableInstance.getDefault().getTable("limelight");
        double tx = table.getEntry("tx").getDouble(0);
        
        // 2026 IMPROVEMENT: Check if a target is actually visible ('tv')
        boolean hasTarget = table.getEntry("tv").getDouble(0) == 1.0;

        /* * 2. Determine if we are ready to fire:
         * - Limelight must see a target (hasTarget)
         * - Shooter must be at the correct RPM (isAtSpeed)
         * - Turret must be centered on target (abs(tx) < 2.0 degrees)
         */
        boolean ready = hasTarget && m_shooter.isAtSpeed(100) && Math.abs(tx) < 2.0;

        if (ready) {
            m_indexer.setPercent(0.8);
            m_intake.setSpeed(.8, -.8);
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        } else {
            m_indexer.stop();
            // Stop rumbling so the driver knows they aren't lined up
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_indexer.stop();
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }
}