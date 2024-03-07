package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ManualClimbCommand extends Command{
    private ClimbSubsystem climbSubsystem;
    private boolean up;

    public ManualClimbCommand(ClimbSubsystem subsystem, boolean up)
    {
        this.climbSubsystem = subsystem;
        this.up = up;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Manual Climb Start");
    }

    @Override
    public void execute() {
        climbSubsystem.setClimbSpeed(up);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopClimb();
        System.out.println("Manual Climb Stop");
    }
}