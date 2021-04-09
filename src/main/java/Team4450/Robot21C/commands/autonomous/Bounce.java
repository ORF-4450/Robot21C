package Team4450.Robot21C.commands.autonomous;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;

import static Team4450.Robot21C.Constants.*;

import Team4450.Robot21C.RobotContainer;
import Team4450.Robot21C.subsystems.DriveBase;
import Team4450.Robot21C.subsystems.Pickup;
import Team4450.Robot21C.commands.NotifierCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This is an autonomous program for the Bounce challenge. It uses our
 * dead reckoning navigation commands.
 */
public class Bounce extends CommandBase
{
    private final DriveBase driveBase;
    private final Pickup    pickup;
	
	private SequentialCommandGroup	commands = null;
    private Command					command = null;
    
    // These constants define the starting pose for this auto program. Defaults to the base starting pose.
    private double                  kInitialX = 1.0, kInitialY = 2.2, kInitialHeading = INITIAL_HEADING;

	/**
	 * Creates a new Bounce autonomous command. This command follows the
     * path for the Bounce challenge.
	 *
	 * @param driveBase DriveBase subsystem used by this command to drive the robot.
	 */
	public Bounce(DriveBase driveBase, Pickup pickup) 
	{
		Util.consoleLog();
		
        this.driveBase = driveBase;
        this.pickup = pickup;
			  
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
		
	  	LCD.printLine(LCD_1, "Mode: Auto - Bounce - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
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
		driveBase.SetCANTalonRampRate(.5);
			
		// Reset odometry tracking with initial x,y position and heading (set above) specific to this 
		// auto routine. Robot must be placed in same starting location each time for pose tracking
		// to work.
		driveBase.resetOdometer(new Pose2d(kInitialX, kInitialY, new Rotation2d()), RobotContainer.navx.getHeading());
		
		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done.
		
		commands = new SequentialCommandGroup();

        command = new AutoDrive(driveBase, .35, 
                                9000, 
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                0);

        commands.addCommands(command);
       
        command = new AutoRotate(driveBase, .35, 270, AutoDrive.Pid.on, AutoDrive.Heading.heading);

        commands.addCommands(command);

        // Drop the pickup to contact the marker.
        command = new NotifierCommand(pickup::toggleDeploy, 0.0, pickup);

        commands.addCommands(command);
        
        command = new AutoDrive(driveBase, .35, 
                                9000, 
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                270);

        commands.addCommands(command);
         
        command = new AutoDrive(driveBase, -.35, 
                                6900, 
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                270);

        commands.addCommands(command);
       
        command = new AutoRotate(driveBase, .35, 45, AutoDrive.Pid.on, AutoDrive.Heading.heading);

        commands.addCommands(command);
         
        command = new AutoDrive(driveBase, .35, 
                                16500, 
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                45);

        commands.addCommands(command);

		command = new AutoCurve2(driveBase, .35, .31, 270, 
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.heading);

        commands.addCommands(command);
         
        command = new AutoDrive(driveBase, .40, 
                                17500, 
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                270);

        commands.addCommands(command);
         
        command = new AutoDrive(driveBase, -.40, 
                                22000, 
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.off,
                                AutoDrive.Pid.on,
                                270);

        commands.addCommands(command);

        command = new AutoRotate(driveBase, .40, 0, AutoDrive.Pid.on, AutoDrive.Heading.heading);

        commands.addCommands(command);
         
        command = new AutoDrive(driveBase, .40, 
                                17000, 
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                0);

        commands.addCommands(command);
       
        command = new AutoRotate(driveBase, .40, 270, AutoDrive.Pid.on, AutoDrive.Heading.heading);

        commands.addCommands(command);
         
        command = new AutoDrive(driveBase, .40, 
                                22000, 
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                270);

        commands.addCommands(command);

		command = new AutoCurve2(driveBase, -.40, -.31, -80, 
                                AutoDrive.StopMotors.stop,
                                AutoDrive.Brakes.on,
                                AutoDrive.Pid.on,
                                AutoDrive.Heading.angle);

        commands.addCommands(command);
		       
		// command = new AutoCurve2(driveBase, .40, .45, -90, 
        //                         AutoDrive.StopMotors.stop,
        //                         AutoDrive.Brakes.on,
        //                         AutoDrive.Pid.on,
        //                         AutoDrive.Heading.angle);
		
		// commands.addCommands(command);
          
		// command = new AutoCurve(driveBase, -.30, .22, -25, 
        //                         AutoDrive.StopMotors.dontStop,
        //                         AutoDrive.Brakes.off,
        //                         AutoDrive.Pid.on,
        //                         AutoDrive.Heading.angle);
		
		// commands.addCommands(command);
        
        // command = new AutoDrive(driveBase, -.30, 
        //                         SRXMagneticEncoderRelative.getTicksForDistance(2, DRIVE_WHEEL_DIAMETER), 
        //                         AutoDrive.StopMotors.stop,
        //                         AutoDrive.Brakes.off,
        //                         AutoDrive.Pid.on,
        //                         AutoDrive.Heading.angle);

        // commands.addCommands(command);
        
		// command = new AutoCurve2(driveBase, -.30, -.42, -140, 
        //                         AutoDrive.StopMotors.stop,
        //                         AutoDrive.Brakes.off,
        //                         AutoDrive.Pid.on,
        //                         AutoDrive.Heading.angle);
		
		// commands.addCommands(command);
         
        // command = new AutoDrive(driveBase, -.30, 
        //                         SRXMagneticEncoderRelative.getTicksForDistance(6, DRIVE_WHEEL_DIAMETER), 
        //                         AutoDrive.StopMotors.stop,
        //                         AutoDrive.Brakes.off,
        //                         AutoDrive.Pid.on,
        //                         AutoDrive.Heading.angle);

        // commands.addCommands(command);
         
        // command = new AutoDrive(driveBase, .30, 
        //                         SRXMagneticEncoderRelative.getTicksForDistance(6, DRIVE_WHEEL_DIAMETER), 
        //                         AutoDrive.StopMotors.stop,
        //                         AutoDrive.Brakes.off,
        //                         AutoDrive.Pid.on,
        //                         AutoDrive.Heading.angle);

        // //commands.addCommands(command);
        
		// command = new AutoCurve(driveBase, .30, .28, -172, 
        //                         AutoDrive.StopMotors.stop,
        //                         AutoDrive.Brakes.off,
        //                         AutoDrive.Pid.on,
        //                         AutoDrive.Heading.angle);
		
		// //commands.addCommands(command);
         
        // command = new AutoDrive(driveBase, .30, 
        //                         SRXMagneticEncoderRelative.getTicksForDistance(6, DRIVE_WHEEL_DIAMETER), 
        //                         AutoDrive.StopMotors.stop,
        //                         AutoDrive.Brakes.off,
        //                         AutoDrive.Pid.on,
        //                         AutoDrive.Heading.angle);

        // //commands.addCommands(command);
        
		// command = new AutoCurve(driveBase, -.30, .29, -95, 
        //                         AutoDrive.StopMotors.stop,
        //                         AutoDrive.Brakes.off,
        //                         AutoDrive.Pid.on,
        //                         AutoDrive.Heading.angle);
		
		// //commands.addCommands(command);
             
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