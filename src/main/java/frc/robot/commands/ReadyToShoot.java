package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hood;

public class ReadyToShoot {
    /**
     * @param distanceSupplier A function that returns the current distance from the Limelight.
     */
    public static Command getCommand(Shooter shooter, Turret turret, Hood hood, DoubleSupplier distanceSupplier) {
        return Commands.parallel(
            // Spin up flywheels
            Commands.run(() -> shooter.setRPM(3500), shooter),
            
            // Track Target with Turret (Corrected Name)
            new TurretTrackTarget(turret),
            
            // Constantly update hood angle based on distance supplier
            Commands.run(() -> hood.setAngleFromDistance(distanceSupplier.getAsDouble()), hood)
        ).finallyDo(() -> {
            // Safety: Stop everything when the command ends/toggle is off
            shooter.stop();
            turret.stop();
            hood.stop();
        });
    }
}