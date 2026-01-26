package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class HopperOut extends Command {
    private final Hopper m_hopper;
    private final double m_distanceInches;

    public HopperOut(Hopper subsystem, double inches) {
        m_hopper = subsystem;
        m_distanceInches = inches;
        addRequirements(m_hopper);
    }

    @Override
    public void initialize() {
        m_hopper.setDistance(m_distanceInches);
    }

    @Override
    public void execute() {
        // Position control is handled on the TalonFXS onboard processor
    }

    @Override
    public boolean isFinished() {
        // Finish when within 0.1 inches of the target
        return m_hopper.isAtPosition(m_distanceInches, 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        // Optional: stop the motor when done
        // m_hopper.stop(); 
    }
}