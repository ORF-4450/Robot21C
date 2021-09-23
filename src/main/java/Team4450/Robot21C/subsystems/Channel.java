package Team4450.Robot21C.subsystems;

import static Team4450.Robot21C.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Channel subsystem.
 */
public class Channel extends SubsystemBase
{
	private boolean			beltRunning;
  	
    private WPI_TalonSRX    beltMotor = new WPI_TalonSRX(BELT_TALON);
      
    private double          defaultPower = .30;

	public Channel()
	{
        //Util.consoleLog();

        beltMotor.setInverted(true);

		Util.consoleLog("Channel created!");
	}
	
	// This method will be called once per scheduler run
	@Override
	public void periodic() 
	{
	}

	private void updateDS()
	{
		SmartDashboard.putBoolean("Belt", beltRunning);
	}

	/**
	 * Stop belt.
	 */
	public void stopBelt()
	{
		Util.consoleLog();
        
        beltMotor.stopMotor();
		
		beltRunning = false;
		
		updateDS();
	}

    /**
	 * Start belt.
	 * @param power Power level -1.0 to 1.0. + is ball IN.
	 */
	public void startBelt(double power)
	{
		Util.consoleLog("%.2f", power);
		
		beltMotor.set(power);
		
		beltRunning = true;
		
		updateDS();
    }
    
    /**
     * Start belt in with default power.
     */
    public void startBelt()
    {
        startBelt(defaultPower);
    }
    
    /**
     * Toggles belt on/off.
     * @param power Power level to use when starting belt level -1.0 to 1.0.
     * + is ball IN.
     * @return True if result is belt on, false if off.
    */
    public boolean toggleBelt(double power)
    {
        //Util.consoleLog("%.2f", power);
        
        if (isRunning())
            stopBelt();
        else
            startBelt(power);

        return isRunning();
    }
    
    /**
     * Toggles belt on/off. Uses default + power level when turning on. Forward
     * is ball IN.
     * @return True if result is belt on, false if off.
    */
    public boolean toggleBeltForward()
    {
        return toggleBelt(defaultPower);
    }
   
    /**
     * Toggles belt on/off. Uses default - power level when turning on. Backward
     * is ball OUT.
     * @return True if result is belt on, false if off.
     */
    public boolean toggleBeltBackward()
    {
        return toggleBelt(-defaultPower);
    }
    
    public Runnable toggleTheBelt(boolean forward)
    {
        Runnable aRunnable = new Runnable() {
            public void run()
            {
               if (forward)
                    toggleBeltForward();
                else
                    toggleBeltBackward();
            }
        };
    
        return aRunnable;
    }
	/**
	 * Returns running state of belt.
	 * @return True is running.
	 */
	public boolean isRunning()
	{
		return beltRunning;
    }
}
