package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class RunIndexer extends Command {
    private final Indexer m_indexer;
    private final double m_speed;

    public RunIndexer(Indexer indexer, double speed) {
        m_indexer = indexer;
        m_speed = speed;
        addRequirements(m_indexer);
    }

    @Override
    public void execute() {
        // We use a positive or negative value based on your hardware setup
        m_indexer.setPercent(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_indexer.stop();
    }
}