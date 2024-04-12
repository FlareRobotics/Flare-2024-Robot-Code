package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_VictorSPX intakeFeederMotor = new WPI_VictorSPX(IntakeConstants.intakeFeederCanID);
    public static final DigitalInput intakeSensor = new DigitalInput(0);
    public double lastSeenTime = 0;

    public IntakeSubsystem() {
        intakeFeederMotor.setInverted(IntakeConstants.intakeFeederReversed);
        intakeFeederMotor.setNeutralMode(IntakeConstants.intakeFeederNeutralMode);
    }


    @Override
    public void periodic() {
        if (Constants.enableSmartDashboard) {
            SmartDashboard.putBoolean("Intake Upper Sensor", getIntakeUpperSensor());
        }
    }

    public Command setIntakeSpeed(double speed)
    {
        return Commands.runOnce(() -> setIntakeSpeedLocal(speed));
    }

    public Command grabNote()
    {
        return Commands.sequence(
            setIntakeSpeed(0.7).alongWith(RobotContainer.SHOOTER_SUBSYSTEM.revertShooterRun()),
            Commands.waitUntil(() -> getIntakeUpperSensor()),
            setIntakeSpeed(-0.25),
            Commands.waitSeconds(0.6),
            stopIntakeMotors().alongWith(RobotContainer.SHOOTER_SUBSYSTEM.stopShooters()));
    }

    private void stopIntakeMotorsLocal()
    {
        intakeFeederMotor.stopMotor();
    }

    public Command stopIntakeMotors()
    {
        return Commands.runOnce(() -> stopIntakeMotorsLocal());
    }
    public void setIntakeSpeedLocal(double speed) {
        intakeFeederMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public static boolean getIntakeUpperSensor() {
        return !intakeSensor.get();
    }
}