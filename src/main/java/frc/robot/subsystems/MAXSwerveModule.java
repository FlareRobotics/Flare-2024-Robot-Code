package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// WPILIB Imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Rev Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;

// CTRE Imports
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

//File Imports
import frc.lib.math.Conversions;
import frc.robot.SwerveConstants;
import frc.robot.SwerveConstants.ModuleConstants;

public class MAXSwerveModule {

  public final TalonFX m_drivingTalon;
  public final CANSparkMax m_turningSparkMax;

  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_turningPIDController;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ModuleConstants.kS, ModuleConstants.kV,
      ModuleConstants.kA);

  private double m_chassisAngularOffset = 0;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean drivingReversed,
      boolean turningReversed) {
    m_drivingTalon = new TalonFX(drivingCANId);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // Same for TalonFX
    // m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    /* Configure TalonFX's Sensor Source for Pirmary PID */
    m_drivingTalon.setInverted(drivingReversed);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(turningReversed);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!(P,I,D,FF)

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingTalon.setNeutralMode(ModuleConstants.kDrivingMotorNeutralMode);
    // No current limit is set for driving motor
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_drivingTalon.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        Conversions.falconToMPS(m_drivingTalon.getVelocity().getValue(), ModuleConstants.kWheelCircumferenceMeters,
            ModuleConstants.kDrivingMotorReduction),
        new Rotation2d(m_turningEncoder.getPosition() + m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        getDriveMotorPosition(),
        new Rotation2d(m_turningEncoder.getPosition() + m_chassisAngularOffset));
  }

  public double getMotorTemp()
  {
    return m_drivingTalon.getDeviceTemp().getValueAsDouble();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */

  SwerveModuleState lastState;

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle       Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

    if (Math.abs(desiredState.speedMetersPerSecond) < SwerveConstants.DriveConstants.kMaxSpeedMetersPerSecond * 0.0012) {
      desiredState.angle = getState().angle;
    }

    desiredState = optimize(desiredState,
        Rotation2d.fromRadians(m_turningEncoder.getPosition() + m_chassisAngularOffset));
    // Command driving and turning SPARKS MAX towards their respective setpoints.

    // Apply chassis angular offset to the desired state.
    // Set speed of Falcon
    setSpeed(desiredState, isOpenLoop);

    m_turningPIDController.setReference(desiredState.angle.getRadians() - m_chassisAngularOffset,
        CANSparkMax.ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingTalon.setPosition(0);
  }

  /**
   * Calculates distance by drive motors encoder.
   *
   * @return Motor position in meters.
   */
  public double getDriveMotorPosition() {
    return Conversions.falconToMeters(m_drivingTalon.getPosition().getValue(),
        ModuleConstants.kWheelCircumferenceMeters, ModuleConstants.kDrivingMotorReduction);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / ModuleConstants.kmaxSpeed;
      m_drivingTalon.set(percentOutput);
    } else {
      double velocity = Conversions.falconToRPM(Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
          ModuleConstants.kWheelCircumferenceMeters, ModuleConstants.kDrivingMotorReduction), 1) / 60.0;
      m_drivingTalon.setControl(new MotionMagicVelocityDutyCycle(velocity, 0, false,
          feedforward.calculate(desiredState.speedMetersPerSecond), 0, true, false, false));
    }
  }
}