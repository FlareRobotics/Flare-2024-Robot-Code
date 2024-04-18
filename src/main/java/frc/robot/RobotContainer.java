package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.commands.Auto.FeedCommand;
import frc.robot.commands.Auto.ShootCommand;
import frc.robot.commands.Climb.ManualClimbCommand;
import frc.robot.commands.IntakeFeeder.AutoIntake;
import frc.robot.commands.LimelightCommands.AutoAlign;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.CustomCommandRunner;

//8054 <3
public class RobotContainer {
        public final static DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem();
        public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
        public final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
        public final ClimbSubsystem CLIMB_SUBSYSTEM = new ClimbSubsystem();

        public static CommandXboxController m_DriverJoy = new CommandXboxController(OIConstants.kDriverControllerPort);
        public static CommandXboxController m_OperatorJoy = new CommandXboxController(
                        OIConstants.kOperatorControllerPort);

        public static RobotState m_RobotState = RobotState.Idle;
        public static boolean m_intaking = false;
        public static SendableChooser<Command> auto_Chooser = new SendableChooser<>();

        boolean driverModeEnabled = false;

        public RobotContainer() {
                // NamedCommands.registerCommand("AutoShoot", generateAutonomousShooterCommand());

                // NamedCommands.registerCommand("GrabNote", generateAutoIntakeCommand());

                // INTAKE_SUBSYSTEM.setDefaultCommand(new AutoIntake(INTAKE_SUBSYSTEM));
                auto_Chooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData(auto_Chooser);

                configureButtonBindings();

                DRIVE_SUBSYSTEM
                                .setDefaultCommand(
                                                new RunCommand(
                                                                () -> DRIVE_SUBSYSTEM
                                                                                .drive(
                                                                                                -MathUtil.applyDeadband(
                                                                                                                m_DriverJoy.getLeftY(),
                                                                                                                OIConstants.kDriveDeadband)
                                                                                                                / 1.00d,
                                                                                                -MathUtil.applyDeadband(
                                                                                                                m_DriverJoy.getLeftX(),
                                                                                                                OIConstants.kDriveDeadband)
                                                                                                                / 1.00d,
                                                                                                MathUtil.applyDeadband(
                                                                                                                m_DriverJoy.getRightX(),
                                                                                                                OIConstants.kDriveDeadband)
                                                                                                                / 2.00d,
                                                                                                true, true, true),
                                                                DRIVE_SUBSYSTEM));
        }

        private void configureButtonBindings() {
                m_OperatorJoy.a()
                                .whileTrue(new RunCommand(() -> INTAKE_SUBSYSTEM.setIntakeSpeed(0.8), INTAKE_SUBSYSTEM)
                                                .alongWith(new InstantCommand(() -> m_intaking = true)))
                                .onFalse(new RunCommand(() -> INTAKE_SUBSYSTEM.setIntakeSpeed(0), INTAKE_SUBSYSTEM)
                                                .alongWith(new InstantCommand(() -> m_intaking = false)));

                m_OperatorJoy.x()
                                .whileTrue(new RunCommand(() -> INTAKE_SUBSYSTEM.setIntakeSpeed(1), INTAKE_SUBSYSTEM))
                                .onFalse(new RunCommand(() -> INTAKE_SUBSYSTEM.setIntakeSpeed(0), INTAKE_SUBSYSTEM));

                m_OperatorJoy.b()
                                .whileTrue(new RunCommand(() -> INTAKE_SUBSYSTEM.setIntakeSpeed(-0.3169),
                                                INTAKE_SUBSYSTEM))
                                .onFalse(new RunCommand(() -> INTAKE_SUBSYSTEM.setIntakeSpeed(0), INTAKE_SUBSYSTEM));

                // Shoot
                m_DriverJoy.rightTrigger()
                                .whileTrue(generateAutoShooterCommand(ShooterConstants.shooterShootRPM))
                                .onFalse(new InstantCommand(
                                                () -> ShooterSubsystem.robotGoalRPM = ShooterConstants.shooterIdleRPM)
                                                .andThen(new InstantCommand(
                                                                () -> IntakeSubsystem.hasNote = false)))
                                .onTrue(new InstantCommand(
                                                () -> IntakeSubsystem.hasNote = true));

                m_DriverJoy.leftTrigger().whileTrue(new AutoAlign(DRIVE_SUBSYSTEM, 0.175))
                .onFalse(new InstantCommand(() -> DRIVE_SUBSYSTEM.drive(0,0,0,false,false,true)));

                // Eject
                m_OperatorJoy.leftTrigger()
                                .whileTrue(generateAutoShooterCommand(800))
                                .onFalse(new InstantCommand(
                                                () -> ShooterSubsystem.robotGoalRPM = ShooterConstants.shooterIdleRPM)
                                                .andThen(new InstantCommand(
                                                                () -> IntakeSubsystem.hasNote = false)))
                                .onTrue(new InstantCommand(
                                                () -> IntakeSubsystem.hasNote = true));

                // Manual Climb
                m_OperatorJoy.leftBumper().whileTrue(new ManualClimbCommand(CLIMB_SUBSYSTEM, false));
                m_OperatorJoy.rightTrigger().whileTrue(new ManualClimbCommand(CLIMB_SUBSYSTEM, true));

                // Zero Heading
                m_OperatorJoy.rightBumper().onTrue(new InstantCommand(() -> DriveSubsystem.zeroHeading(false)));

                // m_OperatorJoy.y().onTrue(new InstantCommand(() -> IntakeSubsystem.intakebozuk
                // = true));
                // m_OperatorJoy.y().whileFalse(new InstantCommand(() ->
                // IntakeSubsystem.intakebozuk = false));

        }

        private void setControllerRumbleOperator(double rumble) {
                m_OperatorJoy.getHID().setRumble(RumbleType.kBothRumble, rumble);
        }

        private void setControllerRumbleDriver(double rumble) {
                m_DriverJoy.getHID().setRumble(RumbleType.kBothRumble, rumble);
        }

        private Command generateAutoShooterCommand(double shooterRPM) {
                return new SequentialCommandGroup(new InstantCommand(
                                () -> ShooterSubsystem.robotGoalRPM = shooterRPM),
                                new InstantCommand(
                                                () -> SHOOTER_SUBSYSTEM.shooterInitialize = false),
                                new WaitUntilCommand(() -> m_RobotState == RobotState.Idle),
                                new InstantCommand(() -> setControllerRumbleDriver(0.8)).withTimeout(0.5)
                                                .andThen(new InstantCommand(() -> setControllerRumbleDriver(0))));
        }

        private Command generateAutonomousShooterCommand() {
                return new ParallelCommandGroup(
                                new InstantCommand(() -> m_intaking = false),
                                new ShootCommand(SHOOTER_SUBSYSTEM),
                                new FeedCommand(INTAKE_SUBSYSTEM, true, false)).withTimeout(1.5)
                                .andThen(new InstantCommand(() -> IntakeSubsystem.hasNote = false));
        }

        private Command generateAutoIntakeCommand() {
                return new CustomCommandRunner(
                                new SequentialCommandGroup(new InstantCommand(() -> m_RobotState = RobotState.Intaking),
                                                new InstantCommand(() -> m_intaking = true),
                                                new InstantCommand(() -> setControllerRumbleOperator(0.8)),
                                                new WaitUntilCommand(() -> m_RobotState != RobotState.Intaking),
                                                new InstantCommand(() -> setControllerRumbleOperator(0)))
                                                .onlyIf(() -> !IntakeSubsystem.hasNote),
                                new SequentialCommandGroup(new InstantCommand(
                                                () -> m_RobotState = (m_RobotState == RobotState.Intaking
                                                                ? RobotState.Idle
                                                                : m_RobotState))))
                                .until(() -> IntakeSubsystem.getIntakeUpperSensor());
        }

        public Command getAutonomousCommand() {

                if (auto_Chooser.getSelected().getName().startsWith("N"))
                        return null;
                return new RunCommand(() -> DRIVE_SUBSYSTEM.drive(0.3, 0, 0, true, false, true), DRIVE_SUBSYSTEM);
        }
}