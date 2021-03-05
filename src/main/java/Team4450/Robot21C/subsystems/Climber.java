package Team4450.Robot21C.subsystems;

import static Team4450.Robot21C.Constants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import Team4450.Robot21C.commands.Traverse;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{
	  
	private WPI_VictorSPX			winchFrontVictor, winchBackVictor, hookVictor;

	private SpeedControllerGroup	winchDrive;
	
	private Traverse				traverseCommand;

	private ValveDA			climberBrake = new ValveDA(CLIMBER_BRAKE_VALVE);
	private DigitalInput	winchSwitch = new DigitalInput(WINCH_SWITCH);

	// Encoder (regular type) is plugged into dio port n:
	// orange=+5v blue=signal, dio port n+1: black=gnd yellow=signal. 
	private Encoder			winchEncoder = new Encoder(WINCH_ENCODER, WINCH_ENCODER + 1, true, EncodingType.k4X);

	private boolean			brakeEngaged;
	
	public Climber(DoubleSupplier traversePower)
	{
		Util.consoleLog();
		
		winchFrontVictor = new WPI_VictorSPX(WINCH_FRONT_VICTOR);
		winchBackVictor = new WPI_VictorSPX(WINCH_BACK_VICTOR);
		//hookVictor = new WPI_VictorSPX(HOOK_VICTOR);
	      
		winchFrontVictor.setInverted(true);
	      
	    winchFrontVictor.setNeutralMode(NeutralMode.Brake);
	    winchBackVictor.setNeutralMode(NeutralMode.Brake);
	    //hookVictor.setNeutralMode(NeutralMode.Brake);

	    winchDrive = new SpeedControllerGroup(winchFrontVictor, winchBackVictor);
	    
	    //traverseCommand = new Traverse(this, traversePower);

		winchEncoder.reset();
		
		releaseBrake();
		
		Util.consoleLog("Climber created!");
	}
	
	@Override
	public void periodic() 
	{
		// This method will be called once per scheduler run
	}
	
	/**
	 * Set power level for climber winch motors.
	 * @param power -1 to +1, + is up because we pull the stick back to climb.
	 */
	public void setWinchPower(double power)
	{
		// If trying to go down (-) and switch returns true, we are at bottom so kill the power.
		
		if (power < 0 && winchSwitch.get()) 
		{
			winchEncoder.reset();
			power = 0;
		}
		
		// If trying to go up (+) and encoder is at upper limit count, we are the top kill the power.
		if (power > 0 && winchEncoder.get() >= 5300) power = 0;

		winchDrive.set(power);
	}
	
	/**
	 * Stops winch motors.
	 */
	public void stop()
	{
		Util.consoleLog();
		
		winchDrive.stopMotor();
		//hookVictor.stopMotor();
	}
	
	/**
	 * Set power level for climber traverse motors.
	 * @param power -1 to +1, + is left when robot viewed from behind.
	 */
	public void setTraversePower(double power)
	{
		hookVictor.set(power);
	}
	
	/**
	 * Engage the climber brake and start traverse mode.
	 */
	public void engageBrake()
	{
		Util.consoleLog();
		
		climberBrake.SetA();
		
		brakeEngaged = true;
		
		// Schedule traverse command which preempts the climb command.
		
		CommandScheduler.getInstance().schedule(traverseCommand);
		
		updateDS();
	}
	
	/**
	 * Release the climber brake and end traverse mode.
	 */
	public void releaseBrake()
	{
		Util.consoleLog();
		
		// End traverse command and climb command resumes.
		
		//traverseCommand.stop();
		
		climberBrake.SetB();
		
		brakeEngaged = false;
		
		updateDS();
	}
	
	/**
	 * Returns state of climber brake.
	 * @return True if  brake engaged.
	 */
	public boolean isBrakeEngaged()
	{
		return brakeEngaged;
	}
	
	/**
	 * Toggle state of climber brake.
	 */
	public void toggleBrake()
	{
		Util.consoleLog();
		
		if (brakeEngaged)
			releaseBrake();
		else
			engageBrake();
	}

	private void updateDS()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Brake", brakeEngaged);
	}

}
