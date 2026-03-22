package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Hood;
import frc.robot.generated.constants;

public class HoodTracking extends Command {
    private final Hood m_hood;
    private final InterpolatingDoubleTreeMap m_angleMap = new InterpolatingDoubleTreeMap();

    public HoodTracking(Hood hood) {
        m_hood = hood;
        addRequirements(m_hood);

        // Map: Distance (m) -> Hood Angle (deg)
        m_angleMap.put(1.5, 45.0);
        m_angleMap.put(2.0, 38.0);
        m_angleMap.put(2.5, 32.0);
        m_angleMap.put(3.0, 25.0);
        m_angleMap.put(3.5, 18.0);
        m_angleMap.put(4.0, 12.0);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("limelight-turret")) { // If a target is visible
            double ty = LimelightHelpers.getTY("limelight-turret");
            
            // Calculate distance using standard trig: (h2-h1) / tan(mountAngle + ty)
            double angleToGoalRadians = Math.toRadians(constants.kLimelightMountAngle + ty);
            double distance = (constants.kTargetHeight - constants.kLimelightHeight) / Math.tan(angleToGoalRadians);

            // Clamp distance and get target angle from map
            distance = MathUtil.clamp(distance, 1.5, 4.0);
            double targetAngle = m_angleMap.get(distance);
            
            m_hood.setAngle(targetAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_hood.stop();
    }
}
