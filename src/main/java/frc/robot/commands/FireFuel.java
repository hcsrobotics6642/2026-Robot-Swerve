package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap; // <-- NEW
import java.util.function.DoubleSupplier; // <-- NEW

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Intake;

public class FireFuel extends Command {
    private final Indexer m_indexer;
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Intake m_intake;
    private final CommandXboxController m_controller;
    
    // NEW: We need a way to get the distance from the RobotContainer
    private final DoubleSupplier m_distanceSupplier; 

    // NEW: Create the Interpolation Map
    private static final InterpolatingDoubleTreeMap m_rpmMap = new InterpolatingDoubleTreeMap();

    // NEW: Add known data points (Distance -> Target RPM)
    // You will need to physically test your robot to find these exact numbers!
    static {
        m_rpmMap.put(5.0, 3000.0);  // At 5 units away, spin at 3000 RPM
        m_rpmMap.put(10.0, 4200.0); // At 10 units away, spin at 4200 RPM
        m_rpmMap.put(15.0, 5100.0); 
        m_rpmMap.put(20.0, 6000.0); // Max RPM for far shots
    }

    public FireFuel(Intake intake, Indexer indexer, Shooter shooter, Turret turret, CommandXboxController controller, DoubleSupplier distanceSupplier) {
        m_indexer = indexer;
        m_shooter = shooter;
        m_turret = turret;
        m_controller = controller;
        m_intake = intake;
        m_distanceSupplier = distanceSupplier; // Save the distance supplier

        // We MUST add m_shooter here so the Command Scheduler knows we are controlling it!
        addRequirements(m_indexer, m_shooter); 
    }

    @Override
    public void execute() {
        // --- NEW: Continuously update shooter RPM based on distance ---
        double distance = m_distanceSupplier.getAsDouble();
        double targetRpm = m_rpmMap.get(distance);
        m_shooter.setRPM(targetRpm);

        // 1. Get Limelight data
        var table = NetworkTableInstance.getDefault().getTable("limelight");
        double tx = table.getEntry("tx").getDouble(0);
        
        // 2026 IMPROVEMENT: Check if a target is actually visible ('tv')
        boolean hasTarget = table.getEntry("tv").getDouble(0) == 1.0;

        /* * 2. Determine if we are ready to fire:
         * - Limelight must see a target (hasTarget)
         * - Shooter must be at the correct RPM (isAtSpeed)
         * - Turret must be centered on target (abs(tx) < 2.0 degrees)
         */
        boolean ready = hasTarget && m_shooter.isAtSpeed(100) && Math.abs(tx) < 2.0;

        if (ready) {
            m_indexer.setPercent(0.8);
            m_intake.setSpeed(.8, -.8);
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        } else {
            // If we aren't lined up or up to speed, STOP feeding the ball!
            // Note: I noticed you had 'm_indexer.stop()', make sure that method exists, 
            // or use m_indexer.setPercent(0.0)
            m_indexer.setPercent(0.0); 
            
            // Stop rumbling so the driver knows they aren't lined up
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Shut everything down when the trigger is released
        m_indexer.setPercent(0.0);
        m_shooter.stop(); // Stop spinning the Kraken x60s to save battery!
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }
}