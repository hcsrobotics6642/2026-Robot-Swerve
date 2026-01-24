package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    private final Intake m_intake;
    private final double m_speed;

    /**
     * Creates a new RunIntake command.
     * @param intake The intake subsystem to use.
     * @param speed The speed to run at (-1.0 to 1.0).
     */
    public RunIntake(Intake intake, double speed) {
        m_intake = intake;
        m_speed = speed;
        
        // Tells the scheduler that this command uses the intake
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        // No setup needed, but you could log a message here
    }

    @Override
    public void execute() {
        // Continuously set the motor to the desired speed
        m_intake.setSpeed(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the button is released OR the command is cancelled
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        // Return false so the command runs as long as the button is held
        return false;
    }
}