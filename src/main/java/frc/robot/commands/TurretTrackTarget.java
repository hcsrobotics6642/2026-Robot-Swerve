package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurretTrackTarget extends Command {
    private final Turret m_turret;

    public TurretTrackTarget(Turret turret) {
        m_turret = turret;
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        // Get horizontal offset from Limelight
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        
        // Simple P-loop for tracking
        double kP = 0.03; 
        double min_command = 0.05;
        
        if (Math.abs(tx) > 1.0) { // Tolerance of 1 degree
            double steering_adjust = kP * tx;
            if (tx > 0) steering_adjust += min_command;
            else steering_adjust -= min_command;
            
            m_turret.setSpeed(steering_adjust);
        } else {
            m_turret.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }
}