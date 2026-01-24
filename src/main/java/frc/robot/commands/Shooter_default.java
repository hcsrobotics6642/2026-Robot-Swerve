package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shooter_default extends Command {
    private final Shooter m_Shooter;
    private final double m_speed;

    public Shooter_default(Shooter shooter, double speed) {
        m_Shooter = shooter;
        m_speed = speed;
        // Use addRequirements so two commands don't try to use the intake at once
        addRequirements(m_Shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (approx 20ms) while the command is active.
    @Override
    public void execute() {
        // 0.9 represents 90% power
        m_Shooter.setMotorSpeed(m_speed); 
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