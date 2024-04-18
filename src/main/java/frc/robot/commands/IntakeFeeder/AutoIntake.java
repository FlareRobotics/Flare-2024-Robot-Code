package frc.robot.commands.IntakeFeeder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntake extends Command {
    private IntakeSubsystem intakeFeederSubsystem;

    public AutoIntake(IntakeSubsystem subsystem) {
        this.intakeFeederSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Auto Intake Start");
    }

    @Override
    public void execute() {
        if (!DriverStation.isAutonomousEnabled())
            return;
        if (RobotContainer.m_RobotState == RobotState.Intaking) {
            intakeFeederSubsystem.setIntakeSpeed(IntakeConstants.intakeGroundSpeedPercentage);
        } else if (RobotContainer.m_RobotState == RobotState.MovingNoteDown) {
            intakeFeederSubsystem.setIntakeSpeed(-IntakeConstants.intakeFeederMovingSpeedPercentage);
        } else if (RobotContainer.m_RobotState == RobotState.ShooterReady) {
            intakeFeederSubsystem.setIntakeSpeed(IntakeConstants.intakeFeederFeedSpeedPercentage);
        } else {
            intakeFeederSubsystem.setIntakeSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Auto Intake End: " + interrupted);
        intakeFeederSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}