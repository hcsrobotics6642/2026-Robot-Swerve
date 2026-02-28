package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hood;
import frc.robot.RobotContainer; // ADD THIS IMPORT

public class ReadyToShoot {
    
    // ADD RobotContainer container to the parameters so we can use its getLimelightAngle method
    public static Command getCommand(Shooter shooter, Turret turret, Hood hood, DoubleSupplier distanceSupplier, RobotContainer container) {
        return Commands.parallel(
            // Spin up flywheels
            Commands.run(() -> shooter.setRPM(3500), shooter),
            
            // Track Target with Turret (Corrected to match TurretTrackTarget constructor)
            new TurretTrackTarget(turret, () -> container.getLimelightAngle(), container),
            
            // Constantly update hood angle based on distance supplier
            Commands.run(() -> hood.setAngleFromDistance(distanceSupplier.getAsDouble()), hood)
        ).finallyDo((interrupted) -> {
            // Safety: Stop everything when the command ends
            shooter.stop();
            turret.stop();
            hood.stop();
        });
    }
}