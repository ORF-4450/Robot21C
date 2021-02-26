package Team4450.Robot21C.subsystems;

import static Team4450.Robot21C.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.Util;
import Team4450.Robot21C.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Turret subsystem.
 */
public class Turret extends SubsystemBase
{
	private boolean			feedRunning;
  	
    private WPI_VictorSPX   feedMotor = new WPI_VictorSPX(TURRET_FEED_VICTOR);
    private WPI_VictorSPX   rotateMotor = new WPI_VictorSPX(TURRET_ROTATE_VICTOR);
      
    private double          defaultFeedPower = .25, defaultRotatePower = .25;

	public Turret()
	{
		//Util.consoleLog();

		Util.consoleLog("Turret created!");
	}
	
	// This method will be called once per scheduler run
	@Override
	public void periodic() 
	{
	}

	private void updateDS()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Feed", feedRunning);
	}

    /**
     * Rotate the turret using default power level.
     * @param power If + rotate right, - rotate left.
     */
    public void rotate(double power)
    {
        if (power > 0)
            rotateMotor.set(defaultRotatePower);
        else if (power < 0)
            rotateMotor.set(-defaultRotatePower);
        else
            rotateMotor.stopMotor();
    }

	/**
	 * Stop feed roller.
	 */
	public void stopFeed()
	{
		Util.consoleLog();
        
        feedMotor.stopMotor();
		
		feedRunning = false;
		
		updateDS();
	}

    /**
	 * Start feed roller.
	 * @param power Power level -1.0 to 1.0.
	 */
	public void startFeed(double power)
	{
		Util.consoleLog();
		
		feedMotor.set(power);
		
		feedRunning = true;
		
		updateDS();
	}
    
    /**
     * Toggles feed roller on/off.
     * @param power Power level to use when starting feed roller -1.0 to 1.0.
      * @return True if result is feed on, false if off.
    */
    public boolean toggleFeed(double power)
    {
        if (isRunning())
            stopFeed();
        else
            startFeed(power);

        return isRunning();
    }
    
    /**
     * Toggles feed roller on/off. Uses default + power level when turning on.
     * @return True if result is feed on, false if off.
    */
    public boolean toggleFeedForward()
    {
        if (isRunning())
            stopFeed();
        else
            startFeed(defaultFeedPower);

        return isRunning();
    }
   
    /**
     * Toggles feed roller on/off. Uses default - power level when turning on.
     * @return True if result is feed on, false if off.
     */
    public boolean toggleFeedBackward()
    {
        if (isRunning())
            stopFeed();
        else
            startFeed(-defaultFeedPower);

        return isRunning();
    }

	/**
	 * Returns running state of feed roller.
	 * @return True is running.
	 */
	public boolean isRunning()
	{
		return feedRunning;
    }

    /**
     * Run the feed roller just long enough to pop the top ball
     * into the shooter wheel. This function should be run from
     * a NotifierCommand so it is run in a separate thread.
     */
    public void feedBall()
    {
        Util.consoleLog();

        // Can't feed a ball if shooter wheel is not running.
        if  (!RobotContainer.shooter.isRunning()) return;

        startFeed(defaultFeedPower);

        Timer.delay(.25);   // set time so one ball is fed.

        stopFeed();
    }
}
