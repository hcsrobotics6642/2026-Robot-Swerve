package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class SetHoodSpeed extends Command {
    private final Hood m_hood;
    private final double m_speed;

    /**
     * Creates a new command to run the hood at a specific speed.
     * * @param hood  The hood subsystem
     * @param speed The target speed (usually -1.0 to 1.0)
     */
    public SetHoodSpeed(Hood hood, double speed) {
        this.m_hood = hood;
        this.m_speed = speed;
        
        // Require the hood so it interrupts any angle commands
        addRequirements(m_hood);
    }

    @Override
    public void initialize() {
        // Optional: Any setup when the command first starts
    }

    @Override
    public void execute() {
        // Continuously apply the speed while the command is running
        m_hood.setMainMotorSpeed(m_speed); 
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motors when the command finishes or is interrupted (e.g., button released)
        m_hood.setMainMotorSpeed(0.0); 
    }

    @Override
    public boolean isFinished() {
        // Returning false means this command runs forever until interrupted.
        // This is perfect for tying to a "whileTrue" button binding.
        return false; 
    }
}