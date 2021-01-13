package Team4450.Robot21C.commands;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;

import static Team4450.Robot21C.Constants.*;
import Team4450.Robot21C.RobotContainer;
import Team4450.Robot21C.subsystems.DriveBase;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAuto1 extends CommandBase
{
	private final DriveBase driveBase;
	
	private SequentialCommandGroup	commands = null;
	private Command					command = null;

	/**
	 * Creates a new TestAuto1 autonomous command. This command demonstrates one
	 * possible structure for an autonomous command and shows the use of the 
	 * autonomous driving support commands.
	 *
	 * @param driveBase DriveBase subsystem used by this command to drive the robot.
	 */
	public TestAuto1(DriveBase driveBase) 
	{
		Util.consoleLog();
		
		this.driveBase = driveBase;
			  
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
		
	  	LCD.printLine(LCD_1, "Mode: Auto - TestAuto1 - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				ds.isFMSAttached(), gameMessage);
		
		// Reset wheel encoders.	  	
	  	driveBase.resetEncodersWithDelay();
	  	
	  	// Set NavX yaw tracking to 0.
	  	RobotContainer.navx.resetYaw();

		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed all during the match.
		RobotContainer.navx.setHeading(0);
			
		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(0);
			
		// Set Talon ramp rate for smooth acceleration from stop. Determine by observation.
		driveBase.SetCANTalonRampRate(1.0);
			
		// Reset odometry tracking with initial x,y position and heading (set above) specific to this 
		// auto routine. Robot must be placed in same starting location each time for pose tracking
		// to work. The settings below are the starting point default for 2021 field.
		driveBase.resetOdometer(new Pose2d(INITIAL_X, INITIAL_Y, new Rotation2d()), RobotContainer.navx.getHeading());
		
		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done.
		
		commands = new SequentialCommandGroup();
		
		// First action is to drive forward some encoder counts and stop with brakes on.
		
		command = new AutoDrive(driveBase, .50, 20000, 
								AutoDrive.StopMotors.stop,
								AutoDrive.Brakes.on,
								AutoDrive.Pid.on,
								AutoDrive.Heading.angle);
		
		command = new AutoDriveProfiled(driveBase, 2.45, AutoDrive.StopMotors.stop, AutoDrive.Brakes.on);
		
		commands.addCommands(command);
		
		// Next action is to rotate 180 degrees left.
		
		//command = new AutoRotate(driveBase, .25, -90, AutoDrive.Pid.on, AutoDrive.Heading.angle);
		
		command = new AutoRotateProfiled(driveBase, -90);

		commands.addCommands(command);
		
		// Next action is to drive back same encoder counts and stop with brakes on.
		
		command = new AutoDrive(driveBase, .50, 10000, 
								AutoDrive.StopMotors.stop,
								AutoDrive.Brakes.on,
								AutoDrive.Pid.on,
								AutoDrive.Heading.angle);

		command = new AutoDriveProfiled(driveBase, 2.45, AutoDrive.StopMotors.stop, AutoDrive.Brakes.on);
		
		commands.addCommands(command);

		//command = new AutoRotate(driveBase, .25, 90, AutoDrive.Pid.on, AutoDrive.Heading.angle);
				
		command = new AutoRotateHdgProfiled(driveBase, 0);

		commands.addCommands(command);
		
		command = new AutoRotateProfiled(driveBase, 90);
		commands.addCommands(command);
		command = new AutoRotateProfiled(driveBase, 90);
		commands.addCommands(command);
		command = new AutoRotateProfiled(driveBase, 90);
		commands.addCommands(command);
		command = new AutoRotateProfiled(driveBase, 90);
		commands.addCommands(command);
		command = new AutoRotateProfiled(driveBase, 90);
		commands.addCommands(command);
		
		command = new AutoCurve(driveBase, .30, .20, 90,
								AutoDrive.StopMotors.stop,
								AutoDrive.Brakes.on,
								AutoDrive.Pid.on,
								AutoDrive.Heading.angle);
		
		//commands.addCommands(command);
		
		// Launch autonomous command sequence.
		
		commands.schedule();
	}
	
	/**
	 *  Called every time the scheduler runs while the command is scheduled.
	 *  In this model, this command just idles while the Command Group we
	 *  created runs on its own executing the steps (commands) of this Auto
	 *  program.
	 */
	@Override
	public void execute() 
	{
	}
	
	/**
	 *  Called when the command ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		driveBase.stop();
		
		Util.consoleLog("final heading=%.2f  Radians=%.2f", RobotContainer.navx.getHeading(), RobotContainer.navx.getHeadingR());
		Util.consoleLog("end -------------------------------------------------------------------------");
	}
	
	/**
	 *  Returns true when this command should end. That should be when
	 *  all the commands in the command list have finished.
	 */
	@Override
	public boolean isFinished() 
	{
		// Note: commands.isFinished() will not work to detect the end of the command list
		// due to how FIRST coded the SquentialCommandGroup class. 
		
		return !commands.isScheduled();
	}
}

