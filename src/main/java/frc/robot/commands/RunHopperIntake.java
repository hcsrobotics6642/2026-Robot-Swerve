package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunHopperIntake extends Command {
    private final Intake m_intake;
    private final double m_speed;

    public RunHopperIntake(Intake intake, double speed) {
        m_intake = intake;
        m_speed = speed;
        
        // Claims the intake subsystem so nothing else conflicts with it
        addRequirements(intake); 
    }

    @Override
    public void execute() {
        // Runs only the 3rd motor (hopper intake)
        m_intake.setHopperIntakeSpeed(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Safely stops ONLY the hopper intake when you let go of the button
        m_intake.setHopperIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until the driver lets go of the button
    }
}