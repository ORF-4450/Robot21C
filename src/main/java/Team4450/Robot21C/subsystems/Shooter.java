package Team4450.Robot21C.subsystems;

import static Team4450.Robot21C.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import Team4450.Lib.FXEncoder;
import Team4450.Lib.Util;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

/**
 * Shooter subsystem.
 */
public class Shooter extends PIDSubsystem
{
	private boolean			wheelRunning;
  	
    private WPI_TalonFX     shooterMotor = new WPI_TalonFX(SHOOTER_TALON);
      
    private FXEncoder       encoder = new FXEncoder(shooterMotor);

    // Green Zone default power  = 0.90
    // Yellow Zone default power = 0.90
    // Blue zone default power   = 0.95
    // Red Zone default power    = 1.00

    private double          defaultPower = 1.00, maxRPM = 6000, targetRPM = 1690, toleranceRPM = 25;
    private double          feedForwardPower = .30;
    private static double   kP = .0001, kI = kP / 100, kD = 0;
    
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(.05, 12 / maxRPM);

	public Shooter()
	{
        super(new PIDController(kP, kI, kD));

        shooterMotor.setInverted(true);
        
        //encoder.setInverted(true);
    
        getController().setTolerance(toleranceRPM);
        
        setSetpoint(targetRPM);
		  
        shooterMotor.setNeutralMode(NeutralMode.Coast);
        
        // Set encoder to update every 100ms.
        encoder.setStatusFramePeriod(100);
        
        Util.consoleLog("Shooter created!");
    }    
	
	// This method will be called once per scheduler run
	@Override
	public void periodic() 
	{
        // Call the base class periodic function so it can run the underlying
        // PID control. We also watch for robot being disabled and stop the
        // wheel (and PID) if running.

        Util.consoleLog("%d  %d", encoder.get(), encoder.getRPM());

        if (robot.isEnabled())
            super.periodic();
        else if (isRunning())
            stopWheel();
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
        
        if (isEnabled()) super.disable(); // Turn off PID if enabled.

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
		Util.consoleLog("%.2f", power);
        
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
        Util.consoleLog("%.2f", power);

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

    //     Util.consoleLog("rpm=%.0f  out=%.3f  set=%.3f  ff=%.3f  v=%.3f", getRPM(), output, setpoint, ff, volts);

    //     shooterMotor.setVoltage(volts);
    // }
    @Override
    protected void useOutput(double output, double setpoint) 
    {
        double feedForwardPower;    

        // Setpoint = 0 means the pid controller is diabled so kill power.

        if (setpoint == 0) 
            feedForwardPower = 0;
        else
            feedForwardPower = this.feedForwardPower;

        //double feedForwardPower = setpoint / maxRPM;

        Util.consoleLog("rpm=%.0f  set=%.0f  ff=%.2f  out=%.3f  pwr=%.3f", getRPM(), setpoint, feedForwardPower, 
                        output, output + feedForwardPower);

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
        Util.consoleLog();

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
}
