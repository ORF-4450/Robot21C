package Team4450.Robot21C.subsystems;

import static Team4450.Robot21C.Constants.SHOOTER_TALON;
import static Team4450.Robot21C.Constants.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import Team4450.Lib.FXEncoder;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

/**
 * Shooter subsystem.
 */
public class Shooter extends PIDSubsystem
{
	private boolean			wheelRunning;
  	
    private WPI_TalonFX     shooterMotor = new WPI_TalonFX(SHOOTER_TALON);
      
    private FXEncoder       encoder = new FXEncoder(shooterMotor);

    private Channel         channel;
    private Turret          turret;

    private final double    greenZonePower  = 0.80, greenZoneRPM = 4850;
    private final double    yellowZonePower = 0.90, yellowZoneRPM = 5400;
    private final double    blueZonePower   = 0.95, blueZoneRPM = 5650;
    private final double    redZonePower    = 1.00, redZoneRPM = 6000;
    
    public final double     defaultPower = greenZonePower, defaultRPM = greenZoneRPM;

    private String          currentZone = "green";
    private double          currentPower = defaultPower, maxRPM = 6000, targetRPM = defaultRPM, toleranceRPM = 50;
    private static double   kP = .00015, kI = kP / 100, kD = 0;
    private boolean         robotEnabled;
    
    // ks and kv determined by characterizing the shooter motor. See the shooter characterization
    // project.
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(.498, .108);

	public Shooter(Channel channel, Turret turret)
	{
        super(new PIDController(kP, kI, kD));

        shooterMotor.setInverted(true);
    
        getController().setTolerance(toleranceRPM);
		  
        shooterMotor.setNeutralMode(NeutralMode.Coast);

        this.channel = channel;
        this.turret = turret;
        
        Util.consoleLog("Shooter created!");
    }    
	
	// This method will be called once per scheduler run including when disabled.
	@Override
	public void periodic() 
	{
        // Call the base class periodic function so it can run the underlying
        // PID control. We also watch for robot being disabled and stop the
        // wheel (and PID) if running.

        if (robot.isEnabled())
        {
            if (!robotEnabled) setZone("green");

            super.periodic();

            robotEnabled = true;
        }
        else
        {
            robotEnabled = false;

            if (isRunning()) stopWheel();
        }
	}

	private void updateDS()
	{
		SmartDashboard.putBoolean("Shooter", wheelRunning);
		SmartDashboard.putString("Zone", currentZone.toUpperCase());
	}

	/**
	 * Stop shooter wheel.
	 */
	public void stopWheel()
	{
		Util.consoleLog();
        
        if (isEnabled()) super.disable(); // Turn off PID if enabled.

        shooterMotor.stopMotor();
		
		wheelRunning = false;
		
		updateDS();
	}

    /**
	 * Start shooter wheel turning. Sets current power level to the
     * level specified.
	 * @param power Power level 0.0 to 1.0.
	 */
	public void startWheel(double power)
	{
		Util.consoleLog("%.2f", power);
        
        currentPower = power;

        if (isEnabled()) disable();  // Turn off underlying PID control. 

        backUpChannel();

		shooterMotor.set(currentPower);
		
		wheelRunning = true;
		
		updateDS();
    }
    
    /**
     * Start shooter wheel turning with current power level.
     */
    public void startWheel()
    {
        startWheel(currentPower);
    }
    
    /**
     * Toggles shooter wheel on/off.
     * @param power Power level to use when starting wheel level 0.0 to 1.0.
     * @return True if result is wheel on, false if off.
     */
    public boolean toggleWheel(double power)
    {
        Util.consoleLog("%.2f", power);

        if (isRunning())
            stopWheel();
        else
            startWheel(power);

        return isRunning();
    }
    
    /**
     * Toggles shooter wheel on/off. Uses current power level when turning on.
     * @return True if result is wheel on, false if off.
     */
    public boolean toggleWheel()
    {
        return toggleWheel(currentPower);
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

    // Called by underlying PID control with the output of the PID calculation
    // each time the scheduler calls the periodic function.
    @Override
    protected void useOutput(double output, double setpoint) 
    {
        // Feed forward calculator wants setpoint in rotations per second so we
        // convert our rpm setpoint to rps. Feed forward calculator should give us
        // a voltage that will achieve the rpm we want. PID controller output will
        // adjust the speed to keeep on the rpm and will compensate for rpm being
        // drug down by a ball passing through the shooter.
        double ff = m_shooterFeedforward.calculate(setpoint / 60);
        
        double volts = output + ff;

        Util.consoleLog("rpm=%.0f  out=%.3f  set=%.3f  ff=%.3f  v=%.3f", getRPM(), output, setpoint, ff, volts);

        shooterMotor.setVoltage(volts);
    }

    // Called by underlying PID control to get the process measurement value each time
    // the scheduler calls the perodic function.
    @Override
    protected double getMeasurement() 
    {
        return getRPM();
    }

    /**
     * Enable PID control to run shooter wheel at current target RPM.
     */
    @Override
    public void enable()
    {
        Util.consoleLog();

        backUpChannel();
        
        setSetpoint(targetRPM);

        super.enable();

        wheelRunning = true;
		
		updateDS();
    }

    /**
     * Enable PID control to run shooter wheel at specified target RPM.
     * Sets current target RPM to the specified value.
     * @param rpm RPM to set PID controller to follow.
     */
    public void enable(double rpm)
    {
        targetRPM = rpm;

        enable();
    }

    /**
     * Disable PID control of shooter wheel.
     */
    @Override
    public void disable()
    {
        Util.consoleLog();

        super.disable();

        stopWheel();
		
		updateDS();
    }
    
    /**
     * Toggles shooter wheel PID  control on/off.
     * @return True if result is wheel on, false if off.
     */
    public boolean togglePID()
    {
        Util.consoleLog();

        if (isEnabled())
           disable();
        else
           enable();

        return isRunning();
    }

    /**
     * Runs channel and feed roller backward before starting shooter motor
     * to move ball out of contact with shooter roller.
     */
    private void backUpChannel()
    {
        Util.consoleLog();

        channel.toggleBeltBackward();
        turret.toggleFeedBackward();

        Timer.delay(.75);   // set time so one ball is fed.

        channel.stopBelt();
        turret.stopFeed();
    }

    /**
     * Change the current shooting zone power/rpm to the next higher level
     * looping around to lowest. 
     */
    public String changeZone()
    {
        switch (currentZone)
        {
            case "green":
                currentZone = "yellow";
                break;

            case "yellow":
                currentZone = "blue";
                break;

            case "blue":
                currentZone = "red";
                break;

            case "red":
                currentZone = "green";
                break;

            default:
                currentZone = "green";
        }

        return setZone(currentZone);
    } 

    /**
     * Change the currentshooting zone power/rpm to the zone requested.
     * @param zone Shooting zone (green/yellow/blue/red).
     * @return
     */
    public String setZone(String zone)
    {
        switch (zone.toLowerCase())
        {
            case "yellow":
                currentPower = yellowZonePower;
                targetRPM = yellowZoneRPM;
                break;

            case "blue":
                currentPower = blueZonePower;
                targetRPM = blueZoneRPM;
                break;

            case "red":
                currentPower = redZonePower;
                targetRPM = redZoneRPM;
                break;

            case "green":
                currentPower = greenZonePower;
                targetRPM = greenZoneRPM;
                break;

            default:
                zone = setZone("green");
        }

        currentZone = zone;

        updateDS();

        return zone;
    }

    public String getCurrentZone()
    {
        return currentZone;
    }
}
