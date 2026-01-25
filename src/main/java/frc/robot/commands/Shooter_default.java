package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shooter_default extends Command {
    private final Shooter m_Shooter;
    private final double m_targetRPM;

    /**
     * Command to run the shooter at a specific RPM
     * @param shooter The shooter subsystem
     * @param targetRPM Desired flywheel speed in rotations per minute (e.g., 3000)
     */
    public Shooter_default(Shooter shooter, double targetRPM) {
        m_Shooter = shooter;
        m_targetRPM = targetRPM;
        addRequirements(m_Shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (approx 20ms) while the command is active.
    @Override
    public void execute() {
        m_Shooter.setRPM(m_targetRPM);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Run until the button is released
    }
}