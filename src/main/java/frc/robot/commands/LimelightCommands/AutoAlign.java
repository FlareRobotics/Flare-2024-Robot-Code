package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlign extends Command {
    DriveSubsystem subsystem;
    boolean isFinished = false;
    double maxDriveSpeed;

    public AutoAlign(DriveSubsystem sDriveSubsystem, double maxDriveSpeed) {
        this.subsystem = sDriveSubsystem;
        this.maxDriveSpeed = maxDriveSpeed;
        addRequirements(sDriveSubsystem);
        subsystem.xPIDController.setSetpoint(0);
        subsystem.yPIDController.setSetpoint(0);
    }

    @Override
    public void initialize() {
        System.out.println("Auto Align Start");
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV("")) {
            RobotContainer.m_DriverJoy.getHID().setRumble(RumbleType.kBothRumble, 0.5);
            return;
        }
        RobotContainer.m_DriverJoy.getHID().setRumble(RumbleType.kBothRumble, 0);

        double horizontalSpeed = subsystem.xPIDController.calculate(
                LimelightHelpers.getTargetPose3d_RobotSpace("").getX(),
                (LimelightHelpers.getTargetPose3d_RobotSpace("").getX() > 0 ? ((Math.abs(DriveSubsystem.getModAngle()) > 300 && Math.abs(DriveSubsystem.getModAngle()) > 40) ? 0.078 : 0) : (Math.abs(DriveSubsystem.getModAngle()) > 300 && Math.abs(DriveSubsystem.getModAngle()) > 40) ? -0.3 : 0));
        double forwardSpeed = subsystem.yPIDController.calculate(
                LimelightHelpers.getTargetPose3d_RobotSpace("").getZ(),
                (LimelightHelpers.getTargetPose3d_RobotSpace("").getX() > 0 ? ((Math.abs(DriveSubsystem.getModAngle()) > 300 && Math.abs(DriveSubsystem.getModAngle()) > 40) ? 1.4 : 1.525) : (Math.abs(DriveSubsystem.getModAngle()) > 300 && Math.abs(DriveSubsystem.getModAngle()) > 40) ? 1.45 : 1.525));

        double rotationalSpeed = subsystem.rotPIDController.calculate(LimelightHelpers.getTX(""), -7);

        forwardSpeed = deSatureSpeeds(forwardSpeed, maxDriveSpeed);
        horizontalSpeed = deSatureSpeeds(horizontalSpeed, maxDriveSpeed);
        rotationalSpeed = deSatureSpeeds(rotationalSpeed, maxDriveSpeed / 1.5);

        subsystem.drive(forwardSpeed, -horizontalSpeed, 0, true, false, true);

        if (subsystem.xPIDController.atSetpoint() && subsystem.yPIDController.atSetpoint()) {
            isFinished = true;
        }
    }

    private double deSatureSpeeds(double speed, double max) {
        if (speed < -max) {
            speed = -max;
        } else if (speed > max) {
            speed = max;
        }

        return speed;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(interrupted);
        System.out.println("Auto Align End");
        isFinished = false;
        RobotContainer.m_DriverJoy.getHID().setRumble(RumbleType.kBothRumble, 0);
        subsystem.drive(0, 0, 0, true, false, true);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}