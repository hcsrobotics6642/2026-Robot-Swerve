package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class L_One_Climb extends Command {
    private final Climber m_climber;
    private final double m_targetRotations;

    public L_One_Climb(Climber subsystem, double inches) {
        m_climber = subsystem;
        
        // Math: (Inches / 3.0 inches per output rotation) * 60.0 gear ratio
        // For 22 inches, this results in 440.0 rotations.
        m_targetRotations = (inches) * 60.0;//Automagically

        addRequirements(m_climber);
    }

    @Override
    public boolean isFinished() {
        // Ends the command when we are within 1 rotation of the goal
        return Math.abs(m_climber.getLeaderPosition() - m_targetRotations) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        // If the command is interrupted (e.g., driver lets go or hits another button)
        if (interrupted) {
            m_climber.stopMotor();
        }
   
    }
@Override
public void initialize() {
    // Safety: Don't move if we are already at the top or bottom limits
    if (m_targetRotations > 0 && m_targetRotations < 500) { 
        m_climber.setClimbPosition(m_targetRotations);
    }
}
}

