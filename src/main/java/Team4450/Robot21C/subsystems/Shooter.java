package Team4450.Robot21C.subsystems;

import static Team4450.Robot21C.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import Team4450.Lib.FXEncoder;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter subsystem.
 */
public class Shooter extends SubsystemBase
{
	private boolean			wheelRunning;
  	
    private WPI_TalonFX     shooterTalon;
      
    private FXEncoder       encoder;

    private double          defaultPower = .25;

	public Shooter()
	{
		Util.consoleLog();

        shooterTalon = new WPI_TalonFX(SHOOTER_TALON);
        
        encoder = new FXEncoder(shooterTalon);

		Util.consoleLog("Shooter created!");
	}
	
	// This method will be called once per scheduler run
	@Override
	public void periodic() 
	{
	}

	private void updateDS()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Shooter", wheelRunning);
	}

	/**
	 * Stop shooter wheel.
	 */
	public void stopWheel()
	{
		Util.consoleLog();
        
        shooterTalon.stopMotor();
		
		wheelRunning = false;
		
		updateDS();
	}

    /**
	 * Start shooter wheel turning.
	 * @param power Power level 0.0 to 1.0.
	 */
	public void startWheel(double power)
	{
		Util.consoleLog();
		
		shooterTalon.set(power);
		
		wheelRunning = true;
		
		updateDS();
	}
    
    /**
     * Toggles shooter wheel on/off.
     * @param power Power level to use when starting wheel level 0.0 to 1.0.
     * @return True if result is wheel on, false if off.
     */
    public boolean toggleWheel(double power)
    {
        if (isRunning())
            stopWheel();
        else
            startWheel(power);

        return isRunning();
    }
    
    /**
     * Toggles shooter wheel on/off. Uses default power level when turning on.
     * @return True if result is wheel on, false if off.
     */
    public boolean toggleWheel()
    {
        if (isRunning())
           stopWheel();
        else
           startWheel(defaultPower);

        return isRunning();
    }

	/**
	 * Returns running state of shooter wheel.
	 * @return True is running.
	 */
	public boolean isRunning()
	{
		return wheelRunning;
    }
    
    /**
     * Returns the current wheel RPM
     * @return The RPM value
     */
    public double getRPM()
    {
        return encoder.getRPM();
    }

    /**
     * Returns the max RPM seen. Depends on regular calls
     * to getRPM.
     * @return The max RPM value.
     */
    public double getMaxRPM()
    {
        return encoder.getMaxRPM();
    }
}
