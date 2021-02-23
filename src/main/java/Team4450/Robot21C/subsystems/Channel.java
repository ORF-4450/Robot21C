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
  	
    private WPI_TalonSRX    beltTalon;
      
    private double          defaultPower = .50;

	public Channel()
	{
		Util.consoleLog();

        beltTalon = new WPI_TalonSRX(SHOOTER_TALON);

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
        
        beltTalon.stopMotor();
		
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
		
		beltTalon.set(power);
		
		beltRunning = true;
		
		updateDS();
	}
    
    /**
     * Toggles belt on/off.
     * @param power Power level to use when starting belt level -1.0 to 1.0.
     */
    public void toggleBelt(double power)
    {
        if (isRunning())
            stopBelt();
        else
            startBelt(power);
    }
    
    /**
     * Toggles belt on/off. Uses default + power level when turning on.
     */
    public void toggleBeltForward()
    {
        if (isRunning())
            stopBelt();
        else
            startBelt(defaultPower);
    }
   
    /**
     * Toggles belt on/off. Uses default - power level when turning on.
     */
    public void toggleBeltBackward()
    {
        if (isRunning())
            stopBelt();
        else
            startBelt(-defaultPower);
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
