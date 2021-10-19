package Team4450.Robot21C.commands.autonomous;

import static Team4450.Robot21C.Constants.INITIAL_HEADING;
import static Team4450.Robot21C.Constants.INITIAL_X;
import static Team4450.Robot21C.Constants.INITIAL_Y;
import static Team4450.Robot21C.Constants.LCD_1;
import static Team4450.Robot21C.Constants.alliance;
import static Team4450.Robot21C.Constants.ds;
import static Team4450.Robot21C.Constants.gameMessage;
import static Team4450.Robot21C.Constants.location;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot21C.RobotContainer;
import Team4450.Robot21C.commands.autonomous.AutoDrive.Brakes;
import Team4450.Robot21C.commands.autonomous.AutoDrive.StopMotors;
import Team4450.Robot21C.subsystems.DriveBase;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This is an example autonomous command based on 4450 customized version of
 * Wpilib Trajetory following commands. If uses a PathWeaver path to drive
 * the barrel racing course.
 */
public class BarrelRacingPath extends CommandBase
{
	private final DriveBase driveBase;
	
	private SequentialCommandGroup	commands = null;
    private Command					command = null;
    
    // These constants define the starting pose for this auto program. Defaults to the base starting pose.
    private double                  kInitialX = INITIAL_X, kInitialY = 2.2, kInitialHeading = INITIAL_HEADING;

	/**
	 * Creates a new BarrelRacingPath autonomous command. This command follows the
     * path for the Barrel Racing challenge.
	 *
	 * @param driveBase DriveBase subsystem used by this command to drive the robot.
	 */
	public BarrelRacingPath(DriveBase driveBase) 
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
		
	  	LCD.printLine(LCD_1, "Mode: Auto - BarrelRacing (Path) - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
				ds.isFMSAttached(), gameMessage);
		
		// Reset wheel encoders.	  	
	  	driveBase.resetEncodersWithDelay();
	  	
	  	// Set NavX yaw tracking to 0.
	  	RobotContainer.navx.resetYaw();

		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed all during the match.
		RobotContainer.navx.setHeading(kInitialHeading);
			
		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(kInitialHeading);
			
		// Set Talon ramp rate for smooth acceleration from stop. Determine by observation.
		driveBase.SetCANTalonRampRate(1.0);
			
		// Reset odometry tracking with initial x,y position and heading (set above) specific to this 
		// auto routine. Robot must be placed in same starting location each time for pose tracking
		// to work.
        driveBase.resetOdometer(new Pose2d(kInitialX, kInitialY, new Rotation2d()), RobotContainer.navx.getHeading());
		
		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done.
		
		commands = new SequentialCommandGroup();
		
        command = new AutoDriveTrajectory(driveBase, RobotContainer.barrel1Trajectory, StopMotors.stop, Brakes.on);

		commands.addCommands(command);
		
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
		Util.consoleLog("end -----------------------------------------------------");
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
