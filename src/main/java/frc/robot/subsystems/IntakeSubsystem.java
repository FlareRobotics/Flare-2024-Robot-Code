package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_VictorSPX intakeFeederMotor = new WPI_VictorSPX(IntakeConstants.intakeFeederCanID);
    public static DigitalInput intakeIRSensor = new DigitalInput(IntakeConstants.intakeIRSensorPort);
    public static  boolean intakebozuk = false;
    public double lastSeenTime = 0;

    public IntakeSubsystem() {
        intakeFeederMotor.setInverted(IntakeConstants.intakeFeederReversed);
        intakeFeederMotor.setNeutralMode(IntakeConstants.intakeFeederNeutralMode);
    }

    public static boolean hasNote = false;
    public static boolean moveNote = false;
    boolean shootingNote = false;

    @Override
    public void periodic() {
        if (Constants.enableSmartDashboard) {
            SmartDashboard.putBoolean("Intake Upper Sensor", getIntakeUpperSensor());
            SmartDashboard.putBoolean("Has Note", hasNote);
            SmartDashboard.putBoolean("Move Note", moveNote);
        }

        if (!DriverStation.isEnabled())
            return;

        if (DriverStation.isAutonomous() && !hasNote && !moveNote) {
            RobotContainer.m_RobotState = RobotState.Intaking;
            RobotContainer.m_intaking = true;
        }

        if (ShooterSubsystem.robotGoalRPM > 0) {
            return;
        }

        if (hasNote && !moveNote) {
            RobotContainer.m_RobotState = RobotState.NoteReady;
        }

        if (getIntakeUpperSensor()) {
            if (ShooterSubsystem.robotGoalRPM <= 0) {
                hasNote = true;
                moveNote = true;
                RobotContainer.m_DriverJoy.getHID().setRumble(RumbleType.kBothRumble, 1);
                RobotContainer.m_RobotState = RobotState.MovingNoteDown;
            } else if (!moveNote) {
                shootingNote = true;
            }
        } else if (moveNote) {
            moveNote = false;
            hasNote = true;
            RobotContainer.m_DriverJoy.getHID().setRumble(RumbleType.kBothRumble, 0);
            RobotContainer.m_RobotState = RobotState.NoteReady;
        } else if (shootingNote) {
            shootingNote = false;
            hasNote = false;
            moveNote = false;

            if (DriverStation.isAutonomous()) {
                RobotContainer.m_RobotState = RobotState.Idle;
            }
        }
    }

    public void setIntakeSpeed(double speed) {
        intakeFeederMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public static boolean getIntakeUpperSensor() {
        return intakeIRSensor.get();
    }
}