package Team4450.Robot21C.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot21C.RobotContainer;
import Team4450.Robot21C.subsystems.DriveBase;
import Team4450.Robot21C.commands.AutoDrive.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoCurve extends CommandBase
{
	private final DriveBase driveBase;

	private double			kP = .04, kI = 0.004, kD = 0.0, kTolerance= 1.0;
	private double			elapsedTime, yaw = 0, originalCurve, power, curve, target;
	private StopMotors 		stop;
	private Brakes 			brakes;
	private Pid 			pid;
	private Heading 		heading;

	SynchronousPID	pidController = null;

	/**
	 * Automatically drive in a curve.
	 * 
	 * @param driveBase The DriveBase subsystem used by this command to drive the robot.
	 * @param power Speed to drive, + is forward.
	 * @param curve Speed of rotation 0..1, always +.
	 * @param target Target angle to turn. If not using heading, this is 0..180, - left, + right. If using heading
	 * this is the target heading 0..359.
	 * @param stop Stop stops motors at end of curve, dontStop leaves power on to flow into next move.
	 * @param brakes On turns on brakes for end of curve, off turns them off.
	 * @param pid On uses PID controller to manage the curve slowing rotation as target is reached. Off
	 * uses the fixed curve value for whole rotation.
	 * @param heading Heading: target is a heading, angle: target is an angle from current direction.
	 * 
	 * Note: This routine is designed for tank drive and the P,I,D,tolerance values will likely need adjusting for each
	 * new drive base as gear ratios and wheel configuration may require different values to stop smoothly
	 * and accurately.
	 */
	public AutoCurve(DriveBase driveBase,
					 double power, 
					 double curve, 
					 double target, 
					 StopMotors stop, 
					 Brakes brakes, 
					 Pid pid, 
					 Heading heading)
	{	
		this.driveBase = driveBase;

		Util.consoleLog("pwr=%.2f  curve=%.2f  target=%.2f  stop=%s  brakes=%s  pid=%s  hdg=%s", 
						power, curve, target, stop, brakes, pid, heading);
		
		Util.checkRange(power, 1.0, "power");
		
		Util.checkRange(curve, 0, 1.0, "curve");

		this.power = power;
		this.curve = curve;
		this.target = target;
		this.stop = stop;
		this.brakes = brakes;
		this.pid = pid;
		this.heading = heading;
		
		//kP = Math.abs(power) / curve;
		//kI = kP / 100.0;
		
		Util.consoleLog("kP=%.5f  kI=%.5f", kP, kI);

		addRequirements(this.driveBase);
	}
	
	@Override
	public void initialize()
	{
		Util.consoleLog();
		
		if (brakes == Brakes.on)
			driveBase.SetCANTalonBrakeMode(true);
		else
			driveBase.SetCANTalonBrakeMode(false);

		// Reset yaw to current robot direction or target heading.
		
		if (heading == Heading.heading) 
		{
			Util.checkRange(target, 0, 359.999, "target");
			
			RobotContainer.navx.setTargetHeading(target);
		}
		else
		{
			Util.checkRange(target, 180, "target");
			
			RobotContainer.navx.resetYawWait(1, 500);
		}
		
		if (pid == Pid.on)
		{
			// Use PID to control power as we turn slowing as we approach target heading.
			
			pidController = new SynchronousPID(kP, kI, kD);
			
			pidController.setOutputRange(-Math.abs(curve) , Math.abs(curve));
			
			originalCurve = curve;
			
			if (heading == Heading.heading)
				pidController.setSetpoint(0);		// We are trying to get the yaw to zero.
			else
				pidController.setSetpoint(target);	// We are trying to get to the target yaw.
			
			// The PID class needs delta time between calls to calculate the I term.
			
			Util.getElaspedTime();
		}
		else if (heading == Heading.heading)		// Simple turn, full curve until target heading reached.
		{
			yaw = RobotContainer.navx.getHeadingYaw();

			if (yaw > 0) curve = curve * -1;
		}
		else										// Simple turn, full curve until target yaw reached.
		{
			yaw = RobotContainer.navx.getYaw();
			
			if (target < 0) curve = curve * -1;
		}
	}
	
	@Override
	public void execute()
	{
		double	power2;
		
		if (pid == Pid.on)
		{
			elapsedTime = Util.getElaspedTime();
			
			if (heading == Heading.heading)
				yaw = RobotContainer.navx.getHeadingYaw();
			else
				yaw = RobotContainer.navx.getYaw();

			// Our target is zero yaw so we determine the difference between
			// current yaw and target and perform the PID calculation which
			// results in the speed (curve) of turn, reducing curve as the difference
			// approaches zero. So our turn should slow and not overshoot. If
			// it does, the PID controller will reverse curve and turn it back.
			
			curve = pidController.calculate(yaw, elapsedTime);
			
			// When quick turn is false, power is constant, curve is fed to the
			// rate of turn parameter. PID controller takes care of the sign, that 
			// is the left/right direction of the turn. If stopping at end of curve
			// we use the PID calculated curve (which is getting smaller) to also
			// reduce motor power for smooth stop.
			
			if (stop == StopMotors.stop)
				power2 = power * Math.abs(curve / originalCurve);
			else
				power2 = power;
			
			driveBase.curvatureDrive(power2, curve, false);
			
			Util.consoleLog("power=%.2f  hdg=%.2f  yaw=%.2f  curve=%.2f  err=%.2f  time=%f", power2, 
					RobotContainer.navx.getHeading(), yaw, curve, pidController.getError(), elapsedTime);			
		}
		else if (heading == Heading.heading)		// Simple turn, full curve until target heading reached.
		{
			driveBase.curvatureDrive(power, curve, false);
			
			Util.consoleLog("yaw=%.2f  hdg=%.2f  curve=%.2f", yaw, RobotContainer.navx.getHeading(), curve);
			
			yaw = RobotContainer.navx.getHeadingYaw();
		}
		else										// Simple turn, full curve until target yaw reached.
		{
			driveBase.curvatureDrive(power, curve, false);
			
			Util.consoleLog("power=%.2f  yaw=%.2f  hdg=%.2f  curve=%.2f", power, yaw, 
							RobotContainer.navx.getHeading(), curve);
			
			yaw = RobotContainer.navx.getYaw();
		}		
	}
	
	@Override
	public void end(boolean interrupted)
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		Util.consoleLog("end loop  hdg=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), yaw);
		
		// Stop motors. If you don't use stop, motors will keep running.
		if (stop == StopMotors.stop) driveBase.stop();

		Util.consoleLog("after stop  hdg=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), yaw);

		// Wait for robot to stop moving.
		Util.consoleLog("moving=%b", RobotContainer.navx.isRotating());
		//while (isAutoActive() && Devices.navx.isRotating()) {Timer.delay(.10);}
		//Util.consoleLog("moving=%b", Devices.navx.isRotating());
		
		Util.consoleLog("end hdg=%.2f  yaw=%.2f ", RobotContainer.navx.getHeading(), yaw);
		Util.consoleLog("end -------------------------------------------------------------------------");
	}
	
	@Override
	public boolean isFinished()
	{
		if (pid == Pid.on)							// pid controlled turn to within x degree of target.
			return pidController.onTarget(kTolerance);
		else if (heading == Heading.heading)		// Simple turn, full curve until target heading reached.
			return Util.checkRange(yaw, 1.0);
		else										// Simple turn, full curve until target yaw reached.
			return Math.abs(yaw) >= Math.abs(target);
	}
}
