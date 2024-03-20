package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private static TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterCanID);
    private TalonFX shooterMotor2 = new TalonFX(ShooterConstants.shooter2CanID);
    public static double robotGoalRPM = 0;
    public boolean shooterInitialize = false;

    private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false,
            false);

    private final VelocityTorqueCurrentFOC m_torqueVelocity2 = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false,
            false);

    public ShooterSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot1.kP = ShooterConstants.shooterMotorKp; // An error of 1 rotation per second results in 5 amps
                                                            // output
        configs.Slot1.kI = ShooterConstants.shooterMotorKi; // An error of 1 rotation per second increases output by 0.1
                                                            // amps every second
        configs.Slot1.kD = ShooterConstants.shooterMotorKd; // A change of 1000 rotation per second squared results in 1
                                                            // amp output

        shooterMotor.getConfigurator().apply(configs);
        shooterMotor2.getConfigurator().apply(configs);

        shooterMotor.setNeutralMode(ShooterConstants.shooterMotorNeutralMode);
        shooterMotor.setInverted(ShooterConstants.shooterMotorReversed);

        shooterMotor2.setNeutralMode(ShooterConstants.shooterMotorNeutralMode);
        shooterMotor2.setInverted(!ShooterConstants.shooterMotorReversed);
    }

    @Override
    public void periodic() {
        if (Constants.enableSmartDashboard) {
            SmartDashboard.putNumber("Shooter RPM",
                    shooterMotor.getVelocity().getValue() * 60);
            SmartDashboard.putNumber("Shooter 2 RPM",
                    shooterMotor2.getVelocity().getValue() * 60);
            SmartDashboard.putNumber("Shooter Temp", shooterMotor.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber("Shooter 2 Temp", shooterMotor2.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber("Shooter Goal RPM", robotGoalRPM);
        }

        if (!DriverStation.isEnabled())
            return;

        if ((DriverStation.isAutonomousEnabled() && IntakeSubsystem.hasNote && !IntakeSubsystem.moveNote)) {
            if (!RobotContainer.auto_Chooser.getSelected().getName().startsWith("I")
                    && !RobotContainer.auto_Chooser.getSelected().getName().startsWith("M")) {
                setShooterRPM(ShooterConstants.shooterShootRPM);
            }
        } else if (shooterInitialize) {
            setShooterRPM(ShooterConstants.shooterShootRPM);
        } else {
            setShooterRPM(robotGoalRPM);
        }

        if (RobotContainer.m_RobotState == RobotState.ShooterReady && robotGoalRPM <= 0)
            RobotContainer.m_RobotState = RobotState.Idle;

        if (getRPMReached(robotGoalRPM)
                && (RobotContainer.m_RobotState == RobotState.NoteReady
                        || RobotContainer.m_RobotState == RobotState.ShooterNotReady
                        || RobotContainer.m_RobotState == RobotState.ShooterReady)) {
            RobotContainer.m_RobotState = RobotState.ShooterReady;
        } else if (RobotContainer.m_RobotState == RobotState.NoteReady) {
            RobotContainer.m_RobotState = RobotState.ShooterNotReady;
        }
    }

    public double convertToPercent(double RPM, double percent) {
        return (RPM / 100) * percent;
    }

    public double comparePercentage(double number1, double number2) {
        if (number1 == 0 || number2 == 0) {
            return -1.0;
        }

        double larger = Math.max(number1, number2);
        double smaller = Math.min(number1, number2);
        double percentage = (larger / smaller) * 100;

        return percentage;
    }

    public void setShooterRPM(double RPM) {
        if (RobotContainer.m_RobotState == RobotState.Intaking
                || RobotContainer.m_RobotState == RobotState.MovingNoteDown) {
            shooterMotor.set(-0.2);
            shooterMotor2.set(-0.2);
            return;
        }

        if (RPM == 0) {
            shooterMotor.set(0);
            shooterMotor2.set(0);
            return;
        }

        shooterMotor.setControl(m_torqueVelocity.withVelocity((RPM / 60)));
        shooterMotor2.setControl(m_torqueVelocity2.withVelocity(convertToPercent((RPM / 60), 100)));
    }

    public static boolean getRPMReached(double goalRPM) {
        if (goalRPM <= 0)
            return false;

        return Math
                .abs((goalRPM - shooterMotor.getVelocity().getValueAsDouble() * 60)) <= ShooterConstants.RPMTolerance;
    }
}