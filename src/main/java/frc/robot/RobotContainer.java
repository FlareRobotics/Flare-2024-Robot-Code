package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.commands.LimelightCommands.AutoAlign;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//8054 <3
public class RobotContainer {
        public final static DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem();
        public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
        public static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
        public final ClimbSubsystem CLIMB_SUBSYSTEM = new ClimbSubsystem();

        public static CommandXboxController m_DriverJoy = new CommandXboxController(OIConstants.kDriverControllerPort);
        public static CommandXboxController m_OperatorJoy = new CommandXboxController(
                        OIConstants.kOperatorControllerPort);

        public static boolean m_intaking = false;
        public static SendableChooser<Command> auto_Chooser = new SendableChooser<>();

        boolean driverModeEnabled = false;

        public RobotContainer() {
                NamedCommands.registerCommand("AutoShoot", SHOOTER_SUBSYSTEM.shootNote());

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

                INTAKE_SUBSYSTEM.setDefaultCommand(INTAKE_SUBSYSTEM.grabNote());
        }

        private void configureButtonBindings() {
                m_DriverJoy.rightTrigger()
                                .whileTrue(new SequentialCommandGroup(new ParallelDeadlineGroup(
                                                Commands.waitUntil(() -> IntakeSubsystem.getIntakeUpperSensor()),
                                                Commands.run(() -> setControllerRumbleDriver(0.8))),
                                                Commands.run(() -> setControllerRumbleDriver(0)),
                                                generatePathOnFlyCommand(),
                                                SHOOTER_SUBSYSTEM.shootNote()))
                                .onFalse(SHOOTER_SUBSYSTEM.stopShooters().alongWith(INTAKE_SUBSYSTEM.stopIntakeMotors()));

                m_DriverJoy.leftTrigger().whileTrue(new SequentialCommandGroup(new ParallelDeadlineGroup(
                                                Commands.waitUntil(() -> IntakeSubsystem.getIntakeUpperSensor()),
                                                Commands.run(() -> setControllerRumbleDriver(0.8))),
                                                Commands.run(() -> setControllerRumbleDriver(0)),
                                                SHOOTER_SUBSYSTEM.ejectNote()))
                                        .onFalse(SHOOTER_SUBSYSTEM.stopShooters().alongWith(INTAKE_SUBSYSTEM.stopIntakeMotors()));

                m_OperatorJoy.x().toggleOnTrue(new SequentialCommandGroup(new ParallelDeadlineGroup(
                                                Commands.waitUntil(() -> !IntakeSubsystem.getIntakeUpperSensor()),
                                                Commands.run(() -> setControllerRumbleOperator(0.8))),
                                                Commands.run(() -> setControllerRumbleOperator(0)),INTAKE_SUBSYSTEM.grabNote().handleInterrupt(() -> INTAKE_SUBSYSTEM.stopIntakeMotors())));

                m_OperatorJoy.rightTrigger().whileTrue(CLIMB_SUBSYSTEM.setClimbSpeed(true));
                m_OperatorJoy.leftTrigger().whileTrue(CLIMB_SUBSYSTEM.setClimbSpeed(false));
        }

        private void setControllerRumbleOperator(double rumble) {
                m_OperatorJoy.getHID().setRumble(RumbleType.kBothRumble, rumble);
        }

        private void setControllerRumbleDriver(double rumble) {
                m_DriverJoy.getHID().setRumble(RumbleType.kBothRumble, rumble);
        }

        private Command generatePathOnFlyCommand() {
                return new AutoAlign(DRIVE_SUBSYSTEM, 0.175);
        }

        public Command getAutonomousCommand() {
                return auto_Chooser.getSelected();
        }
}