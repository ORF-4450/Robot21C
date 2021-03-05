package Team4450.Robot21C.commands.autonomous;

import Team4450.Lib.FXEncoder;
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

/**
 * This is an example autonomous command based on 4450 customized version of
 * Wpilib Trajetory following commands.
 */
public class AutoSlalom2 extends CommandBase
{
	private final DriveBase driveBase;
	
	private SequentialCommandGroup	commands = null;
    private Command					command = null;
    
    // These constants define the starting pose for this auto program. Defaults to the base starting pose.
    private double                  kInitialX = INITIAL_X, kInitialY = INITIAL_Y, kInitialHeading = INITIAL_HEADING;

	/**
	 * Creates a new AutoSlalom autonomous command. This command follows the
     * path for the Slalom challenge.
	 *
	 * @param driveBase DriveBase subsystem used by this command to drive the robot.
	 */
	public AutoSlalom2(DriveBase driveBase) 
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
		
	  	LCD.printLine(LCD_1, "Mode: Auto - AutoSlalom2 - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
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

		command = new AutoCurve(driveBase, .30, .30, -60,
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
		
		commands.addCommands(command);
        
        command = new AutoDrive(driveBase, .50, FXEncoder.getTicksForDistance(9 * 12, DRIVE_WHEEL_DIAMETER), 
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);

        commands.addCommands(command);

		command = new AutoCurve(driveBase, .50, .16, 120,
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
		
		commands.addCommands(command);
        
        command = new AutoDrive(driveBase, .50, FXEncoder.getTicksForDistance(9 * 12, DRIVE_WHEEL_DIAMETER), 
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);

        commands.addCommands(command);

		command = new AutoCurve(driveBase, .50, .35, -180,
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
		
		commands.addCommands(command);

		command = new AutoCurve(driveBase, .50, .35, -125,
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
		
		commands.addCommands(command);
        
        command = new AutoDrive(driveBase, .50, FXEncoder.getTicksForDistance(5 * 12, DRIVE_WHEEL_DIAMETER), 
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);

        commands.addCommands(command);

		command = new AutoCurve(driveBase, .50, .25, 64,
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
		
		commands.addCommands(command);
        
        command = new AutoDrive(driveBase, .50, FXEncoder.getTicksForDistance(11 * 12, DRIVE_WHEEL_DIAMETER), 
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);

        commands.addCommands(command);

		command = new AutoCurve(driveBase, .50, .25, 50,
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
		
		commands.addCommands(command);
        
        command = new AutoDrive(driveBase, .50, FXEncoder.getTicksForDistance(8 * 12, DRIVE_WHEEL_DIAMETER), 
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);

        commands.addCommands(command);

		command = new AutoCurve(driveBase, .50, .25, -50,
                                AutoDrive.StopMotors.dontStop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);
		
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
