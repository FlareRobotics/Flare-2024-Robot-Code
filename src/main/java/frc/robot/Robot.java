package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.math.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer = new RobotContainer();

  @SuppressWarnings("resource")
  @Override
  public void robotInit() {
    Logger.recordMetadata("Flare 2024", "Main");
    Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
    new PowerDistribution(0, ModuleType.kCTRE);

   // Logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void disabledInit() {
    ShooterSubsystem.robotGoalRPM = 0;
    RobotContainer.m_RobotState = RobotState.Idle;
    DriveSubsystem.setBrake(false);
    DriveSubsystem.setWheelsZeroPos();
  }

  @Override
  public void autonomousExit() {
    ShooterSubsystem.robotGoalRPM = 0;
    RobotContainer.m_RobotState = RobotState.Idle;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (Constants.enableSmartDashboard) {
      SmartDashboard.putBoolean("Vision Has Targets", LimelightHelpers.getTV(""));
      SmartDashboard.putNumber("AprilTag ID", LimelightHelpers.getFiducialID(""));
      SmartDashboard.putNumber("Vision Latency",
          LimelightHelpers.getLatency_Pipeline("") +
              LimelightHelpers.getLatency_Capture(""));
    }

    if (!DriverStation.isTeleopEnabled()) {
      RobotContainer.m_DriverJoy.getHID().setRumble(RumbleType.kBothRumble, 0);
      RobotContainer.m_OperatorJoy.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    if (!DriverStation.isEnabled()) {
      if (LimelightHelpers.getTV("")) {
        LimelightHelpers.setLEDMode_ForceBlink("");
      } else {
        LimelightHelpers.setLEDMode_ForceOff("");
      }
    }
    else {
      LimelightHelpers.setLEDMode_ForceOff("");
    }
  }

  @Override
  public void autonomousInit() {
    if(DriverStation.getAlliance().get()== Alliance.Blue){
      DriveSubsystem.zeroHeading(false);
    }else{
      DriveSubsystem.zeroHeading(false);
    }
    LimelightHelpers.setLEDMode_ForceOff("");
    IntakeSubsystem.hasNote = true;
    int[] ids = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    LimelightHelpers.SetFiducialIDFiltersOverride("", ids);
    DriveSubsystem.setBrake(true);
    DriveSubsystem.resetEncoders();
    

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    int[] ids = {4,7};
    LimelightHelpers.SetFiducialIDFiltersOverride("", ids);
    DriveSubsystem.setBrake(true);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}