package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
public class FeedCommand extends Command {
    private IntakeSubsystem intakeFeederSubsystem;
    private boolean waitForRPM = false;

    public FeedCommand(IntakeSubsystem subsystem, boolean waitForRPM) {
        this.intakeFeederSubsystem = subsystem;
        this.waitForRPM = waitForRPM;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Auto Feed Start");
    }


    @Override
    public void execute() {
        if(ShooterSubsystem.getRPMReached(ShooterConstants.shooterShootRPM) || !waitForRPM){
            intakeFeederSubsystem.setIntakeSpeed(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Auto Feed End: " + interrupted);
        intakeFeederSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}