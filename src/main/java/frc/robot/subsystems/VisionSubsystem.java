package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.LimelightHelpers;
import frc.lib.math.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.RobotContainer;
import frc.robot.SwerveConstants.DriveConstants;

public class VisionSubsystem extends SubsystemBase{
  private final DriveSubsystem swerve;
  public SwerveDrivePoseEstimator poseEst;

  private final Field2d field2d;

  public VisionSubsystem(DriveSubsystem swerve) {
    this.swerve = swerve;
    field2d = new Field2d();

    poseEst = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(swerve.getHeading()),
        new SwerveModulePosition[] {
            DriveSubsystem.m_frontLeft.getPosition(),
            DriveSubsystem.m_frontRight.getPosition(),
            DriveSubsystem.m_rearLeft.getPosition(),
            DriveSubsystem.m_rearRight.getPosition() },
        new Pose2d());
  }

  public void periodic() {

    field2d.setRobotPose(getCurrentPose());

    poseEst.update(Rotation2d.fromDegrees(swerve.getHeading()), new SwerveModulePosition[] {
        DriveSubsystem.m_frontLeft.getPosition(),
        DriveSubsystem.m_frontRight.getPosition(),
        DriveSubsystem.m_rearLeft.getPosition(),
        DriveSubsystem.m_rearRight.getPosition() });

    SmartDashboard.putData("Field Pose Est", field2d);

    var blueRightResult = LimelightHelpers.getLatestResults("").targetingResults;

    Pose2d blueRightBotPose = new Pose2d(blueRightResult.getBotPose2d_wpiBlue().getTranslation(), Rotation2d.fromDegrees(swerve.getHeading()));

    double rightTimestamp = Timer.getFPGATimestamp() - (blueRightResult.latency_capture / 1000.0)
        - (blueRightResult.latency_pipeline / 1000.0);

    if (blueRightResult.targets_Fiducials.length > 0 && (!RobotContainer.auto_Chooser.getSelected().getName().startsWith("M") || !DriverStation.isAutonomous())) {
      if (getAvgTA(blueRightResult.targets_Fiducials) > 0.0050) {
        poseEst.addVisionMeasurement(blueRightBotPose, rightTimestamp);
      } 
    }

  }

  public Pose2d getCurrentPose() {
    return poseEst.getEstimatedPosition();
  }

  public double getAvgTA(LimelightTarget_Fiducial[] fiducials) {
    double sumTA = 0;
    for (int i = 0; i < fiducials.length; i++) {
      sumTA += fiducials[i].ta;
    }
    return sumTA / fiducials.length;
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEst.resetPosition(Rotation2d.fromDegrees(swerve.getHeading()), new SwerveModulePosition[] {
        DriveSubsystem.m_frontLeft.getPosition(),
        DriveSubsystem.m_frontRight.getPosition(),
        DriveSubsystem.m_rearLeft.getPosition(),
        DriveSubsystem.m_rearRight.getPosition() }, newPose);
  }
}