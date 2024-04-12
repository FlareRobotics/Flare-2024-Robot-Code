package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.LimelightHelpers;
import frc.lib.math.LimelightHelpers.PoseEstimate;
import frc.lib.math.LimelightHelpers.RawFiducial;
import frc.robot.RobotContainer;
import frc.robot.SwerveConstants.DriveConstants;

public class VisionSubsystem extends SubsystemBase {
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
    LimelightHelpers.SetRobotOrientation("", swerve.getHeading(), 0, 0, 0, 0, 0);
    field2d.setRobotPose(getCurrentPose());

    poseEst.update(Rotation2d.fromDegrees(swerve.getHeading()), new SwerveModulePosition[] {
        DriveSubsystem.m_frontLeft.getPosition(),
        DriveSubsystem.m_frontRight.getPosition(),
        DriveSubsystem.m_rearLeft.getPosition(),
        DriveSubsystem.m_rearRight.getPosition() });

    SmartDashboard.putData("Field Pose Est", field2d);

    SmartDashboard.putNumber("X Lime", LimelightHelpers.getTargetPose3d_CameraSpace("").getX());
    SmartDashboard.putNumber("Z Lime", LimelightHelpers.getTargetPose3d_CameraSpace("").getZ());
    SmartDashboard.putNumber("TX Lime", LimelightHelpers.getTX(""));

    PoseEstimate blueRightBotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

    if (Math.abs(DriveSubsystem.m_gyro.getRate()) > 720)
      return;

    if (blueRightBotPose.rawFiducials.length > 0
        && (!RobotContainer.auto_Chooser.getSelected().getName().startsWith("M") || !DriverStation.isAutonomous())) {
      if (getAvgTA(blueRightBotPose.rawFiducials) > 0.0025) {
        poseEst.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        poseEst.addVisionMeasurement(blueRightBotPose.pose, blueRightBotPose.timestampSeconds);
      }
    }
  }

  public Pose2d getCurrentPose() {
    return poseEst.getEstimatedPosition();
  }

  public double getAvgTA(RawFiducial[] fiducials) {
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