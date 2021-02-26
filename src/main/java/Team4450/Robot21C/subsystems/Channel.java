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
      
    private double          defaultPower = .25;

	public Channel()
	{
		//Util.consoleLog();

		Util.consoleLog("Channel created!");
	}
	
	// This method will be called once per scheduler run
	@Override
	public void periodic() 
	{
	}

	private void updateDS()
	{
		Util.consoleLog();

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
	 * @param power Power level -1.0 to 1.0.
	 */
	public void startBelt(double power)
	{
		Util.consoleLog();
		
		beltMotor.set(power);
		
		beltRunning = true;
		
		updateDS();
	}
    
    /**
     * Toggles belt on/off.
     * @param power Power level to use when starting belt level -1.0 to 1.0.
      * @return True if result is belt on, false if off.
    */
    public boolean toggleBelt(double power)
    {
        if (isRunning())
            stopBelt();
        else
            startBelt(power);

        return isRunning();
    }
    
    /**
     * Toggles belt on/off. Uses default + power level when turning on.
     * @return True if result is belt on, false if off.
    */
    public boolean toggleBeltForward()
    {
        return toggleBelt(defaultPower);
    }
   
    /**
     * Toggles belt on/off. Uses default - power level when turning on.
     * @return True if result is belt on, false if off.
     */
    public boolean toggleBeltBackward()
    {
        return toggleBelt(-defaultPower);
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
