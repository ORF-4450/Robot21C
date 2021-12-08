package Team4450.Robot21C.subsystems;

import static Team4450.Robot21C.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.Util;
import Team4450.Robot21C.RobotContainer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
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
    
    private AnalogInput     limitSensorLeft = new AnalogInput(TURRET_LIMIT_LEFT);
    private AnalogInput     limitSensorRight = new AnalogInput(TURRET_LIMIT_RIGHT);
    
    private double          defaultFeedPower = .25, defaultRotatePower = .20;

    private Channel         channel;

	public Turret(Channel channel)
	{
        Util.consoleLog();
        
        this.channel = channel;

        rotateMotor.setInverted(true);
        feedMotor.setInverted(true);

        Util.consoleLog("Turret created!");
	}
	
	// This method will be called once per scheduler run
	@Override
	public void periodic() 
	{
        updateDS();    
	}

    public void stop()
    {
        Util.consoleLog();

        feedMotor.stopMotor();
        rotateMotor.stopMotor();
    }

	private void updateDS()
	{
        SmartDashboard.putBoolean("Feed", feedRunning);
        SmartDashboard.putNumber("TurretLimitLeft", limitSensorLeft.getValue());      
        SmartDashboard.putNumber("TurretLimitRight", limitSensorRight.getValue());      
	}

    /**
     * Rotate the turret using default power level. Optical sensors on turrent
     * prevent over rotation. Input power is only used to determine direction of
     * rotation. We use fixed power to keep turret movement at a fixed slow rate.
     * @param power If + rotate right, - rotate left.
     */
    public void rotate(double power)
    {
        // Sensors are analog and return above 400 when not blocked by turret rotation.

        if (power > 0 && limitSensorLeft.getValue() > 400)
            rotateMotor.set(defaultRotatePower);
        else if (power < 0 && limitSensorRight.getValue() > 400)
            rotateMotor.set(-defaultRotatePower);
        else
            rotateMotor.stopMotor();
    }

    /**
     * Rotate the turret using specified power level. Optical sensors on turrent
     * prevent over rotation. Given that the power level can vary, it is possible
     * to rotate fast enough to defeat the optical sensors and over rotate. This
     * function is intended for use by the auto aiming comand.
     * @param power If + rotate right, - rotate left.
     */
    public void rotateVariable(double power)
    {
        // Sensors return true when not blocked by turret rotation.

        if (power > 0 && limitSensorLeft.getValue() > 400)
            rotateMotor.set(power);
        else if (power < 0 && limitSensorRight.getValue() > 400)
            rotateMotor.set(power);
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
		Util.consoleLog("%.2f", power);
		
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
        Util.consoleLog("%.2f", power);

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
        Util.consoleLog();

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
        Util.consoleLog();

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

        channel.startBelt();

        Timer.delay(.50);   // set time so one ball is fed.

        channel.stopBelt();

        stopFeed();
    }
}
