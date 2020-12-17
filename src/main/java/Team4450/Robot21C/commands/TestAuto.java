package Team4450.Robot21C.commands;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;

import static Team4450.Robot21C.Constants.*;
import Team4450.Robot21C.RobotContainer;
import Team4450.Robot21C.subsystems.DriveBase;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestAuto extends CommandBase
{
	private final DriveBase driveBase;

	/**
	 * Creates a new TestAuto autonomous command.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public TestAuto(DriveBase subsystem) 
	{
		Util.consoleLog();
		
		driveBase = subsystem;
			  
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(this.driveBase);
	}
	
	/**
	 * Called when the command is initially scheduled. (non-Javadoc)
	 * @see edu.wpi.first.wpilibj2.command.Command#initialize()
	 */
	@Override
	public void initialize() 
	{
		Util.consoleLog();
		
		driveBase.setMotorSafety(false);  // Turn off watchdog.
		
	  	LCD.printLine(LCD_1, "Mode: Auto - TestAutoCommand - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				ds.isFMSAttached(), gameMessage);
		
		// Reset wheel encoders.	  	
	  	driveBase.resetEncodersWithDelay();
	  	
	  	// Set NavX yaw tracking to 0.
	  	RobotContainer.navx.resetYaw();

		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed during the match.
		RobotContainer.navx.setHeading(0);
			
		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(0);
			
		// Set Talon ramp rate for smooth acceleration from stop. Determine by observation.
		driveBase.SetCANTalonRampRate(1.0);
			
		// Reset odometry tracking with initial x,y position and heading (set above) specific to this 
		// auto routine. Robot must be placed in same starting location each time for pose tracking
		// to work. The settings below are the starting point default for 2021 field.
		driveBase.resetOdometer(new Pose2d(INITIAL_X, INITIAL_Y, new Rotation2d()), RobotContainer.navx.getHeading());
	}
	
	/**
	 *  Called every time the scheduler runs while the command is scheduled.
	 */
	@Override
	public void execute() 
	{
		Util.consoleLog();
	}
	
	/**
	 *  Called when the command ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		driveBase.stop();
	}
	
	/**
	 *  Returns true when the command should end.
	 */
	@Override
	public boolean isFinished() 
	{
		return false;	// false makes this command run until auto mode ends.
	}
}
