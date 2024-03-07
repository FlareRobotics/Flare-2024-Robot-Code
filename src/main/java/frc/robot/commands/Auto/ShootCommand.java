package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    private ShooterSubsystem shooterSubsystem;

    public ShootCommand(ShooterSubsystem subsystem)
    {
        this.shooterSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Auto Shooter Start");
        ShooterSubsystem.robotGoalRPM = ShooterConstants.shooterShootRPM;
    }

    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.robotGoalRPM = ShooterConstants.shooterIdleRPM;
        shooterSubsystem.setShooterRPM(0);
        IntakeSubsystem.hasNote = false;
        System.out.println("Auto Shooter Manual End");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}