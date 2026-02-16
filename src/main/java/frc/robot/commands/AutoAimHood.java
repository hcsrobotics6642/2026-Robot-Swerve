package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class AutoAimHood extends Command {
    private final Hood m_hood;

    public AutoAimHood(Hood hood) {
        m_hood = hood;
        addRequirements(m_hood);
    }

    @Override
    public void execute() {
        // 1. Get distance from Limelight (requires 'ty' and camera height/angle setup)
        // This is a placeholder for your distance calculation logic
        double distance = getLimelightDistance(); 

        // 2. Tell the hood to look up the correct angle for that distance
        m_hood.setAngleFromDistance(distance);
    }

    private double getLimelightDistance() {
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        // Standard FRC distance formula: (GoalHeight - CameraHeight) / tan(CameraAngle + ty)
        return (2.6 - 0.5) / Math.tan(Math.toRadians(30.0 + ty));
    }

    @Override
    public void end(boolean interrupted) {
        m_hood.stop();
    }
}