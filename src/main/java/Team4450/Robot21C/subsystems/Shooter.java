package Team4450.Robot21C.subsystems;

import static Team4450.Robot21C.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import Team4450.Lib.FXEncoder;
import Team4450.Lib.Util;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Shooter subsystem.
 */
public class Shooter extends PIDSubsystem
{
	private boolean			wheelRunning;
  	
    private WPI_TalonFX     shooterMotor = new WPI_TalonFX(SHOOTER_TALON);
      
    private FXEncoder       encoder = new FXEncoder(shooterMotor);

    private double          defaultPower = .75, maxRPM = 6200, targetRPM = 2000, toleranceRPM = 200;
    private double          feedForwardPower = 1.0;
    private static double   kP = .0002, kI = kP / 100, kD = 0;
    
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(.05, 12 / maxRPM);

	public Shooter()
	{
        super(new PIDController(kP, kI, kD));

        shooterMotor.setInverted(true);
    
        getController().setTolerance(toleranceRPM);
        
        setSetpoint(targetRPM);
		  
        shooterMotor.setNeutralMode(NeutralMode.Coast);

        Util.consoleLog("Shooter created!");
    }    
	
	// This method will be called once per scheduler run
	@Override
	public void periodic() 
	{
        // Call the base class periodic function so it can run the underlying
        // PID control.
        super.periodic();
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
        
        if (isRunning()) super.disable(); // Turn off PID if enabled.

        shooterMotor.stopMotor();
		
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
        
        disable();  // Turn off underlying PID control. 

		shooterMotor.set(power);
		
		wheelRunning = true;
		
		updateDS();
    }
    
    /**
     * Start shooter wheel turning with default power.
     */
    public void startWheel()
    {
        startWheel(defaultPower);
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
        return toggleWheel(defaultPower);
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
    // @Override
    // protected void useOutput(double output, double setpoint) 
    // {
    //     double ff = m_shooterFeedforward.calculate(setpoint);
    //     double volts = (output * 12) + ff;

    //     Util.consoleLog("out=%.3f  set=%.3f  ff=%.3f  v=%.3f", output, setpoint, ff, volts);

    //     shooterMotor.setVoltage(volts);
    // }
    @Override
    protected void useOutput(double output, double setpoint) 
    {
        Util.consoleLog("out=%.3f  set=%.3f  ff=%.3f  rpm=%.0f", output, setpoint, feedForwardPower, getRPM());

        // Set motor to feed forward power adjusted by PID result capped at +-1.0.
        shooterMotor.set(Util.clampValue(output + feedForwardPower, 1.0));
    }

    // Called by underlying PID control to get the process measurement value each time
    // the scheduler calls the perodic function.
    @Override
    protected double getMeasurement() 
    {
        return getRPM();
    }

    /**
     * Enable PID control to run shooter wheel at target RPM.
     */
    @Override
    public void enable()
    {
        super.enable();

        wheelRunning = true;
		
		updateDS();
    }

    /**
     * Disable PID control of shooter wheel.
     */
    @Override
    public void disable()
    {
        super.disable();

        wheelRunning = false;
		
		updateDS();
    }
    
    /**
     * Toggles shooter wheel PID  control on/off.
     * @return True if result is wheel on, false if off.
     */
    public boolean togglePID()
    {
        if (isRunning())
           disable();
        else
           enable();

        return isRunning();
    }
}
