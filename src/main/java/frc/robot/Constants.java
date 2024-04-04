package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Constants 
{
    public static boolean enableSmartDashboard = true;

    public static final class IntakeConstants
    {
        public static int intakeFeederCanID = 7;
        public static int intakeFeederSensorEndID = 0;
        public static double intakeUpperSensorDistance = 30;
        public static double intakeGroundSpeedPercentage = 1;
        public static double intakeFeederMovingSpeedPercentage = 0.25;
        public static double intakeFeederFeedSpeedPercentage = 1;
        public static boolean intakeFeederReversed = false;
        public static NeutralMode intakeFeederNeutralMode = NeutralMode.Brake;
    }

    public static final class ShooterConstants
    {
        public static int shooterCanID = 0;
        public static int shooter2CanID = 1;
        public static int shooterIdleRPM = 0;
        public static double shooterShootRPM = 4800;
        public static boolean shooterMotorReversed = false;
        public static NeutralModeValue shooterMotorNeutralMode = NeutralModeValue.Brake;
        public static double shooterMotorKp = 5;
        public static double shooterMotorKi = 2;
        public static double shooterMotorKd = 0;
        public static double RPMTolerance = 50;
    }

    public static final class ClimbConstants
    {
        public static int climbCanID1 = 2;
        public static int climbCanID2 = 5;
        public static boolean climbMotor1Reversed = false;
        public static boolean climbMotor2Reversed = true;
        public static NeutralMode climbMotorNeutralMode = NeutralMode.Brake;
        public static double climbSpeedPercentage = 0.85;
    }
}