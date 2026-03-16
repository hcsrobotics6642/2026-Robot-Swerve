package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class EjectHopper extends Command {
    private final Hopper m_hopper;
    private final double m_speed;

    /**
     * Runs the hopper ejector motor at the specified speed.
     * * @param hopper The Hopper subsystem
     * @param speed  The speed to run the ejector (-1.0 to 1.0)
     */
    public EjectHopper(Hopper hopper, double speed) {
        m_hopper = hopper;
        m_speed = speed;
        
        // This is crucial: it tells the robot that no other command 
        // can use the Hopper while this one is running!
        addRequirements(hopper);
    }

    @Override
    public void execute() {
        // Runs repeatedly while the button is held
        m_hopper.setSpeed(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        // The instant you let go of the button, stop the motor!
        m_hopper.stop();
    }

    @Override
    public boolean isFinished() {
        // Returning false means this command runs forever until interrupted (button release)
        return false; 
    }
}