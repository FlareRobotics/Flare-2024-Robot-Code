package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwerveConstants.AutoConstants;
import frc.robot.SwerveConstants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  public static final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      true, true);

  public static final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      true, true);

  public static final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      true, true);

  public static final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      true, true);

  // The gyro sensor
  public static final Pigeon2 m_gyro = new Pigeon2(DriveConstants.gyroCanId);

  private SlewRateLimiter m_magXLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter m_magYLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAcceleration);

  public PIDController xPIDController = new PIDController(0.4, 0, 0);
  public PIDController yPIDController = new PIDController(0.4, 0, 0);
  public PIDController rotPIDController = new PIDController(0.1, 0, 0);

  public boolean shifterEnabled = false;

  public final VisionSubsystem vision = new VisionSubsystem(this);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_drive_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getHeading()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    resetEncoders();
    zeroHeading(false);

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(AutoConstants.pathDrivekP, 0.0, 0.0), // Translation PID constants
            new PIDConstants(AutoConstants.pathTurnkP, 0.001, 0.0), // Rotation PID constants
            4.8, // Max module speed, in m/s
            .415, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> DriverStation.getAlliance().get() == Alliance.Blue,
        this // Reference to this subsystem to set requirements
    );

    xPIDController.setTolerance(0.03);
    yPIDController.setTolerance(0.03);
    rotPIDController.setTolerance(2);
  }

  public static double getModAngle()
  {
    return m_gyro.getAngle() % 360;
  }

  @Override
  public void periodic() {
    if (Constants.enableSmartDashboard) {
      SmartDashboard.putNumber("Gyro Angle", getModAngle());
      SmartDashboard.putString("Robot State", RobotContainer.m_RobotState.toString());
    }

    // Update the odometry in the periodic block
    m_drive_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    SmartDashboard.putNumber("X Lime", LimelightHelpers.getTargetPose3d_RobotSpace("").getX());
    SmartDashboard.putNumber("Z Lime", LimelightHelpers.getTargetPose3d_RobotSpace("").getZ());
    SmartDashboard.putNumber("TX Lime", LimelightHelpers.getTX(""));

    Logger.recordOutput("Encoder Only Odometry", m_drive_odometry.getPoseMeters());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return vision.poseEst.getEstimatedPosition();
  }

  public Pose2d getOdomPose() {
    return m_drive_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_drive_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

        vision.setCurrentPose(pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit,
      boolean isOpenLoop) {

    double xSpeedDelivered, ySpeedDelivered, rotDelivered;
    if (rateLimit) {
      // Convert the commanded speeds into the correct units for the drivetrain
      xSpeedDelivered = m_magXLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = m_magYLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = m_rotLimiter.calculate(rot) * DriveConstants.kMaxAngularSpeed;
    } else {
      xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    }

    if(shifterEnabled)
    {
      xSpeedDelivered /= 2;
      ySpeedDelivered /= 2;
    }
    
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getHeading() + 180))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0], isOpenLoop);
    m_frontRight.setDesiredState(swerveModuleStates[1], isOpenLoop);
    m_rearLeft.setDesiredState(swerveModuleStates[2], isOpenLoop);
    m_rearRight.setDesiredState(swerveModuleStates[3], isOpenLoop);
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    speeds.omegaRadiansPerSecond *= -1;
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0], true);
    m_frontRight.setDesiredState(swerveModuleStates[1], true);
    m_rearLeft.setDesiredState(swerveModuleStates[2], true);
    m_rearRight.setDesiredState(swerveModuleStates[3], true);
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
        m_rearLeft.getState(), m_rearRight.getState());
  }

  public static void setBrake(boolean enabled) {
    m_frontLeft.m_drivingTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    m_frontRight.m_drivingTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    m_rearLeft.m_drivingTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    m_rearRight.m_drivingTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);

    m_frontLeft.m_turningSparkMax.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    m_frontRight.m_turningSparkMax.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    m_rearLeft.m_turningSparkMax.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    m_rearRight.m_turningSparkMax.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public static void setWheelsZeroPos() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public static void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0], isOpenLoop);
    m_frontRight.setDesiredState(desiredStates[1], isOpenLoop);
    m_rearLeft.setDesiredState(desiredStates[2], isOpenLoop);
    m_rearRight.setDesiredState(desiredStates[3], isOpenLoop);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public static void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public static void zeroHeading(Boolean reverse) {
    if(reverse){
      m_gyro.setYaw(180);
    }else{
    m_gyro.setYaw(0);
    }
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getYaw().getValueAsDouble();
  }
}