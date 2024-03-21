package frc.robot;

//CTRE IMPORTS
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class SwerveConstants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAcceleration = 1.85;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kMaxAngularAcceleration = 4;

    // Chassis configuration
    public static final double kTrackWidth = 0.61;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.40;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = -Math.PI / 2;

    // Drivetrain motorcontroller ID's
    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftDrivingCanId = 9;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 19;
    public static final int kRearLeftTurningCanId = 14;
    public static final int kFrontRightTurningCanId = 13;
    public static final int kRearRightTurningCanId = 18;

    public static final int gyroCanId = 7;
    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // Constants for Feedforward from SYSID
    public static final double kS = 0.034;
    public static final double kV = 0.3;
    public static final double kA = 0.018;
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Max speed of one module with 14T gear and Falcon 500 in m/s
    public static final double kmaxSpeed = 4.8;
    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 6380 / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = ((45.0 * 22) / (kDrivingMotorPinionTeeth * 15));
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final com.revrobotics.CANSparkBase.IdleMode kTurningMotorIdleMode = com.revrobotics.CANSparkBase.IdleMode.kBrake;
    public static final NeutralModeValue kDrivingMotorNeutralMode = NeutralModeValue.Brake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 30; // amps
  }

  public static final class OIConstants {

    public static final int kDriverControllerPort = 3;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.08;
  }

  public static final class AutoConstants {
    public static final double pathDrivekP = 3;
    public static final double pathTurnkP = 0.95;
  }
}