package Team4450.Robot21C.commands;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;

import static Team4450.Robot21C.Constants.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import Team4450.Robot21C.RobotContainer;
import Team4450.Robot21C.commands.AutoDrive.Brakes;
import Team4450.Robot21C.commands.AutoDrive.StopMotors;
import Team4450.Robot21C.subsystems.DriveBase;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This is an example autonomous command based on 4450 customized version of
 * Wpilib Trajetory following commands.
 */
public class TestAuto3 extends CommandBase
{
	private final DriveBase driveBase;
	
	private SequentialCommandGroup	commands = null;
    private Command					command = null;
    
    // These constants define the starting pose for this auto program. Defaults to the base starting pose.
    private double                  kInitialX = INITIAL_X, kInitialY = INITIAL_Y, kInitialHeading = INITIAL_HEADING;

	/**
	 * Creates a new TestAuto1 autonomous command. This command demonstrates one
	 * possible structure for an autonomous command and shows the use of the 
	 * autonomous driving support commands.
	 *
	 * @param driveBase DriveBase subsystem used by this command to drive the robot.
	 */
	public TestAuto3(DriveBase driveBase) 
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
		
	  	LCD.printLine(LCD_1, "Mode: Auto - TestAuto3 - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location, 
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
		Pose2d startPose = driveBase.resetOdometer(new Pose2d(kInitialX, kInitialY, new Rotation2d()), RobotContainer.navx.getHeading());
		
		// Since a typical autonomous program consists of multiple actions, which are commands
		// in this style of programming, we will create a list of commands for the actions to
		// be taken in this auto program and add them to a sequential command list to be 
		// executed one after the other until done.
		
		commands = new SequentialCommandGroup();
		
		// We will create a trajectory and set the robot to follow it.
    
        DifferentialDriveVoltageConstraint constraint = AutoDriveTrajectory.getVoltageConstraint();

        TrajectoryConfig config = AutoDriveTrajectory.getTrajectoryConfig(constraint);

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //                                 // Start at the origin set above
        //                                 startPose,
        //                                 // Pass through these two interior waypoints, making an 's' curve path
        //                                 List.of(
        //                                     new Translation2d(startPose.getX() + 3, startPose.getY() + 1),
        //                                     new Translation2d(startPose.getX() + 6, startPose.getY() + 1)
        //                                 ),
        //                                 // End 4 meters straight ahead of where we started, facing forward
        //                                 new Pose2d(startPose.getX() + 9, startPose.getY(), startPose.getRotation()),
        //                                 // Pass config
        //                                 config);		

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                        // Start at the origin set above
                                        startPose,
                                        // Pass through these two interior waypoints, making an 's' curve path
                                        List.of(
                                            new Translation2d(startPose.getX() + 1, startPose.getY()),
                                            new Translation2d(startPose.getX() + 2, startPose.getY())
                                        ),
                                        // End 4 meters straight ahead of where we started, facing forward
                                        new Pose2d(startPose.getX() + 3, startPose.getY(), startPose.getRotation()),
                                        // Pass config
                                        config);		
        
        exampleTrajectory = loadTrajectoryFile("Slalom-1.wpilib.json");

		command = new AutoDriveTrajectory(driveBase, exampleTrajectory, StopMotors.stop, Brakes.on);
		
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
    
    /**
     * Loads a Pathweaver path file into a trajectory.
     * @param fileName Name of file. Will automatically look in deploy directory.
     * @return The path's trajectory.
     */
    private Trajectory loadTrajectoryFile(String fileName)
    {
        Trajectory  trajectory;
        Path        trajectoryFilePath;

        try 
        {
          trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + fileName);
          
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryFilePath);
        } catch (IOException ex) {
          throw new RuntimeException("Unable to open trajectory: " + ex.toString());
        }

        Util.consoleLog("trajectory loaded: %s", trajectoryFilePath);

        return trajectory;
    }
}
