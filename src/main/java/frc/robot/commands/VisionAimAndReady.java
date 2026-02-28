package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import java.util.function.DoubleSupplier;

public class VisionAimAndReady extends Command {
    private final Shooter m_shooter;
    private final Hood m_hood;
    private final Turret m_turret;
    private final DoubleSupplier m_distanceSupplier;

    // MATCHES FIRE FUEL COMMAND EXACTLY
    private static final InterpolatingDoubleTreeMap m_rpmMap = new InterpolatingDoubleTreeMap();

    static {
        m_rpmMap.put(5.0, 3000.0);
        m_rpmMap.put(10.0, 4200.0);
        m_rpmMap.put(15.0, 5100.0);
        m_rpmMap.put(20.0, 6000.0);
    }

    public VisionAimAndReady(Shooter shooter, Hood hood, Turret turret, DoubleSupplier distanceSupplier) {
        m_shooter = shooter;
        m_hood = hood;
        m_turret = turret;
        m_distanceSupplier = distanceSupplier;
        
        // Ensure we control all three subsystems so nothing else interferes
        addRequirements(m_shooter, m_hood, m_turret);
    }

    @Override
    public void execute() {
        // 1. Get Limelight data
        var table = NetworkTableInstance.getDefault().getTable("limelight");
        double tx = table.getEntry("tx").getDouble(0);
        boolean hasTarget = table.getEntry("tv").getDouble(0) == 1.0;
        
        double distance = m_distanceSupplier.getAsDouble();

        // 2. Aim the Turret
        if (hasTarget) {
            // If the target is offset by 'tx' degrees, we subtract it from our current angle 
            // to command the turret to center itself. (You may need to change the minus to a plus 
            // depending on your physical encoder/motor inversion!)
            double targetAngle = m_turret.getAngle() - tx;
            m_turret.setAngle(targetAngle);
        } else {
            // Optional: If we lose the target, stop moving the turret
            m_turret.setSpeed(0);
        }

        // 3. Spool up the Shooter & angle the Hood
        if (distance > 0) {
            // Calculate exact RPM using the interpolation map
            double targetRpm = m_rpmMap.get(distance);
            m_shooter.setRPM(targetRpm);
            
            // Set Hood angle based on distance
            m_hood.setAngleFromDistance(distance);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Safe Shutdown when the button is released
        m_shooter.stop();
        m_turret.setSpeed(0);
        
        // Optional: Return the hood to a "safe" resting angle (e.g., 0 degrees) so it 
        // doesn't get smashed by another robot driving by!
        m_hood.setAngle(0); 
    }

    // Factory method for easy binding in RobotContainer
    public static Command getCommand(Shooter shooter, Turret turret, Hood hood, DoubleSupplier distance) {
        return new VisionAimAndReady(shooter, hood, turret, distance);
    }
}