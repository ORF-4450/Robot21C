package Team4450.Robot21C.subsystems;

import static Team4450.Robot21C.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.InterruptHandlerFunction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pickup extends SubsystemBase
{
	private WPI_TalonSRX	pickupTalon;

	private ValveDA			pickupValve = new ValveDA(PICKUP_VALVE);

	private DigitalInput	ballEye = new DigitalInput(BALL_EYE);

	private boolean			extended = false, pickupRunning = false;
    public static boolean   balleye = false;
    
	public Pickup ()
	{
		Util.consoleLog();

		pickupTalon = new WPI_TalonSRX(PICKUP_TALON);
		  
		pickupTalon.setInverted(true);

		InitializeCANTalon(pickupTalon);
		
		// Configure interrupt handler for the ballEye optical ball detector.
		
		ballEye.requestInterrupts(new InterruptHandler());
		
		// Listen for a falling edge interrupt.
		
		ballEye.setUpSourceEdge(true, false);

		retract();
		
		Util.consoleLog("Pickup created!");
	}
	
	@Override
	public void periodic() 
	{
		// This method will be called once per scheduler run
	}

	private void updateDS()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Pickup", pickupRunning);
		SmartDashboard.putBoolean("PickupExtended", extended);
	}
	
	/**
	 * Extend the pickup arm and start the wheel motor.
	 * Method is thread safe.
	 */
	public void extend()
	{
		Util.consoleLog();
		
		synchronized (this)
		{
			pickupValve.SetA();
			
			extended = true;
			
			start(.50);
		}
	}
	
	/**
	 * Retract the pickup arm and stop the wheel motor.
	 * Method is thread safe.
	 */
	public void retract()
	{
		Util.consoleLog();
		
		synchronized (this)
		{
			pickupValve.SetB();
			
			extended = false;
			
			stop();
		}
	}
	  
	/**
	 * Toggle between pickup arm extended and retracted.
	 */
	public void toggleDeploy()
	{
		Util.consoleLog("%b", isExtended());
		
		if (isExtended())
			retract();
		else
		  	extend();
	}
	
	/**
	 * Start pick up wheel and enable optical sensor interrupts.
	 * @param power % power to run wheel motor 0.0->1.0.
	 */
	private void start(double power)
	{
		Util.consoleLog("%.2f", power);
		
		pickupTalon.set(power);
		
		pickupRunning = true;
		
		ballEye.enableInterrupts();
		
		updateDS();
	}

	/**
	 * Stop wheel motor and disable interrupts.
	 */
	private void stop()
	{
		Util.consoleLog();
		
		pickupTalon.stopMotor();
	
		pickupRunning = false;
		
		// Note, the following function is expensive in terms of the
		// time it takes and will trigger the global watchdog warning
		// and the drivebase motor safety will trigger if set below 1
		// second. No idea why this function takes so long.
		
		ballEye.disableInterrupts();
	
		updateDS();
	}
	
	/**
	 * Returns extended state of pickup arm.
	 * @return True if arm extended.
	 */
	public boolean isExtended()
	{
		return extended;
	}
	
	/**
	 * Returns state of wheel motor.
	 * @return True if running.
	 */
	public boolean isRunning()
	{
		return pickupRunning;
	}
		
	// Interrupt handler object to handle detection of ball by the ball Eye
	// optical sensor. The sensor generates a hardware interrupt when the 
	// eye is triggered and the fired method is called when interrupt occurs.
	// Keep the length of the code in that method short as no new interrupts
	// will be reported until fired method ends.
	
	private class InterruptHandler extends InterruptHandlerFunction<Object> 
	{
	     @Override
	     public void interruptFired(int interruptAssertedMask, Object param) 
	     {
	    	 Util.consoleLog("ball  interrupt");
	    	 
	    	 //channel.intakeBall();
	    	 
	    	 //Channel channel = (Channel) param;
	    	 //channel.intakeBall();
	     }
	     
//		 public Channel overridableParamter()
//	     {
//			return channel;
//	     }
	}
	
	/**
	 * Returns state of ball detector electric eye.
	 * @return True means ball blocking eye. Will only be true when ball passing eye.
	 */
	public boolean getBallEye()
	{

		//balleye = !balleye;
		//return balleye;
		return !ballEye.get();
	}

	// Initialize and Log status indication from CANTalon. If we see an exception
	// or a talon has low voltage value, it did not get recognized by the RR on start up.
	  
	private static void InitializeCANTalon(WPI_TalonSRX talon)
	{
		Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

		talon.clearStickyFaults(0); //0ms means no blocking.
	}
}
