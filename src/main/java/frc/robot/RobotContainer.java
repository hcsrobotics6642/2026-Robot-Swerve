package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
    /* Drive Constants */
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    /* Swerve Requests */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Controllers */
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final Hopper m_hopper = new Hopper();
    private final Climber m_climber = new Climber();
    private final Turret m_turret = new Turret();
    private final Hood m_hood = new Hood();
    private final Indexer m_indexer = new Indexer();

    /* Restored Command Groups */
    SequentialCommandGroup Start_Match = new SequentialCommandGroup(
        new HopperOut(m_hopper, 12));

    SequentialCommandGroup Shoot = new SequentialCommandGroup(
        new Shooter_default(m_shooter, 4500));

    SequentialCommandGroup Climb = new SequentialCommandGroup(
        new L_Three_Climb(m_climber, 22.0),
        new WaitCommand(0.5),
        new L_Three_Climb(m_climber, -5.0));

    public RobotContainer() {
        configureBindings();
    }

    private double getLimelightDistance() {
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        return (2.6 - 0.5) / Math.tan(Math.toRadians(30.0 + ty));
    }

    private double getClimberTagOffset() {
        return NetworkTableInstance.getDefault().getTable("limelight-climber").getEntry("tx").getDouble(0);
    }

    private void configureBindings() {
        /* --- PATHPLANNER / NAMED COMMANDS --- */
        NamedCommands.registerCommand("START", Start_Match);
        NamedCommands.registerCommand("Shoot", Shoot);
        NamedCommands.registerCommand("Climb", Climb);
        
        // New Vision Commands for Auto
        NamedCommands.registerCommand("AutoPrep", ReadyToShoot.getCommand(m_shooter, m_turret, m_hood, this::getLimelightDistance));
        NamedCommands.registerCommand("AlignToTower", 
            drivetrain.applyRequest(() -> drive.withRotationalRate(getClimberTagOffset() * -0.05))
            .until(() -> Math.abs(getClimberTagOffset()) < 1.0).withTimeout(2.0));

        /* --- DRIVER CONTROLS (Port 0) --- */
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                     .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                     .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)
            )
        );

        // Reset Heading
        m_driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Drive Team Intake (Parallel logic for indexer)
        m_driverController.leftTrigger().whileTrue(Commands.parallel(
            new RunIntake(m_intake, -0.9, 0.9),
            Commands.startEnd(() -> m_indexer.setPercent(-0.4), m_indexer::stop, m_indexer)
        ));

        // Point wheels (restored from original)
        m_driverController.start().and(m_driverController.b()).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));

        // SysId Routines (restored from original)
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        /* --- OPERATOR CONTROLS (Port 1) --- */
        
        // Scoring Logic
        m_operatorController.start().toggleOnTrue(
            ReadyToShoot.getCommand(m_shooter, m_turret, m_hood, this::getLimelightDistance)
        );
        m_operatorController.rightTrigger().whileTrue(new FireFuel(m_intake, m_indexer, m_shooter, m_turret, m_operatorController));

        // Manual Setpoints
        m_operatorController.a().onTrue(new SetHoodAngle(m_hood, 15.0));
        m_operatorController.y().onTrue(new SequentialCommandGroup(
            new L_Three_Climb(m_climber, 22.0),
            new WaitCommand(0.5),
            new L_Three_Climb(m_climber, 0.0),
            new WaitCommand(0.5),
            new L_Three_Climb(m_climber, 22.0)
        ));
        m_operatorController.b().onTrue(new InstantCommand(() -> m_turret.setAngle(90.0), m_turret));
        m_operatorController.x().whileTrue(new RunIntakeOut(m_intake, 0.9, -0.9));

        /* --- SYSTEM UTILITIES --- */
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0)).withTimeout(5.0),
            drivetrain.applyRequest(() -> idle)
        );
    }
}