package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import java.util.function.DoubleSupplier;

public class VisionAimAndReady extends Command {
    private final Shooter m_shooter;
    private final Hood m_hood;
    private final Turret m_turret;
    private final DoubleSupplier m_distanceSupplier;

    public VisionAimAndReady(Shooter shooter, Hood hood, Turret turret, DoubleSupplier distanceSupplier) {
        m_shooter = shooter;
        m_hood = hood;
        m_turret = turret;
        m_distanceSupplier = distanceSupplier;
        addRequirements(m_shooter, m_hood, m_turret);
    }

    @Override
    public void execute() {
        double distance = m_distanceSupplier.getAsDouble();
        if (distance > 0) {
            m_shooter.setRPM(3000 + (distance * 120));
            m_hood.setAngleFromDistance(distance);
        }
    }

    // This "Factory" method allows RobotContainer to call it easily
    public static Command getCommand(Shooter shooter, Turret turret, Hood hood, DoubleSupplier distance) {
        return new VisionAimAndReady(shooter, hood, turret, distance);
    }
}