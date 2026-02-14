package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    private final Intake m_intake;
    private final double m_speed;
    private final double m_Rspeed;

    /**
     * Creates a new RunIntake command.
     * @param intake The intake subsystem to use.
     * @param speed The front motor speed (-1.0 to 1.0).
     * @param Rspeed The rear motor speed (-1.0 to 1.0).
     */
    public RunIntake(Intake intake, double speed, double Rspeed) {
        m_intake = intake;
        m_speed = speed;
        m_Rspeed = Rspeed;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_intake.setSpeed(m_speed, m_Rspeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}