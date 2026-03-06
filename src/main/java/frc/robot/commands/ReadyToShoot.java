package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap; // <-- NEW
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hood;
import frc.robot.RobotContainer; 

public class ReadyToShoot {
    
    // 1. CREATE THE AIMBOT MAP
    private static final InterpolatingDoubleTreeMap m_rpmMap = new InterpolatingDoubleTreeMap();

    // 2. ADD YOUR PLACEHOLDER DATA POINTS
    static {
        // Format: m_rpmMap.put(Distance_In_Feet, Target_RPM);
        m_rpmMap.put(5.0, 2000.0);  // Close shot
        m_rpmMap.put(10.0, 3500.0); // Medium shot
        m_rpmMap.put(15.0, 4500.0); // Far shot
        m_rpmMap.put(20.0, 5500.0); // Maximum range
    }

    public static Command getCommand(Shooter shooter, Turret turret, Hood hood, DoubleSupplier distanceSupplier, RobotContainer container) {
        return Commands.parallel(
            
            // 3. CONTINUOUSLY UPDATE SHOOTER RPM BASED ON DISTANCE
            Commands.run(() -> {
                double currentDistance = distanceSupplier.getAsDouble();
                // .get() mathematically calculates the exact RPM for the current distance!
                double calculatedRPM = m_rpmMap.get(currentDistance); 
                shooter.setRPM(calculatedRPM);
            }, shooter),
            
            // Track Target with Turret 
           // new TurretTrackTarget(turret, () -> container.getLimelightAngle(), container),
            
            // Constantly update hood angle based on distance supplier
            Commands.run(() -> hood.setAngleFromDistance(distanceSupplier.getAsDouble()), hood)

        ).finallyDo((interrupted) -> {
            // Safety: Stop everything when the driver lets go of the button
            shooter.stop();
            turret.stop();
            hood.stop();
        });
    }
}