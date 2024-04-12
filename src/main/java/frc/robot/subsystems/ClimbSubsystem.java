package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/*Will not be used??
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
*/

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{

    private WPI_VictorSPX climbMotor = new WPI_VictorSPX(ClimbConstants.climbCanID1);
    private WPI_VictorSPX climbMotor2 = new WPI_VictorSPX(ClimbConstants.climbCanID2);

    public ClimbSubsystem()
    {
        climbMotor.setInverted(ClimbConstants.climbMotor1Reversed);
        climbMotor.setNeutralMode(ClimbConstants.climbMotorNeutralMode);

        climbMotor2.setInverted(ClimbConstants.climbMotor2Reversed);
        climbMotor2.setNeutralMode(ClimbConstants.climbMotorNeutralMode);        
    }

    public void stopMotor()
    {
        climbMotor.set(VictorSPXControlMode.PercentOutput, 0);
        climbMotor2.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public Command setClimbSpeed(boolean up)
    {
        return Commands.runOnce(() -> setClimbSpeedLocal(up));
    }

    public void setClimbSpeedLocal(boolean up)
    {
        climbMotor.set(VictorSPXControlMode.PercentOutput, ClimbConstants.climbSpeedPercentage * (up ? 1 : -1));
        climbMotor2.set(VictorSPXControlMode.PercentOutput, ClimbConstants.climbSpeedPercentage * (up ? 1 : -1));
    }

    public void stopClimb()
    {
        stopMotor();
    }
}