
package Team4450.Robot21C;

import Team4450.Lib.*;

import Team4450.Robot21C.subsystems.ColorWheel;
import static Team4450.Robot21C.Constants.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  private RobotContainer	robotContainer;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit()
  {
	  try
	  {
		  robot = this;

	   	  LCD.clearAll();
	   	  LCD.printLine(LCD_1, "Mode: RobotInit");

	   	  // Set up our custom logger.

	   	  Util.CustomLogger.setup();
			
		  // Set Java to catch any uncaught exceptions and record them in our log file. 
			
		  Thread.setDefaultUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() 
		  {
			  public void uncaughtException(Thread t, Throwable e)
			  {
				  Util.consoleLog("Uncaught exception from thread " + t);
			      Util.logException(e);
			   }
		  });
		
		  // Eliminate LW overhead when not using it.
		  LiveWindow.disableAllTelemetry();
		  
		  // Create SendableVersion object so it can be sent to the dashboard and also
		  // log some of it's information.
			
		  SendableVersion.INSTANCE.init(PROGRAM_NAME);
			
		  Util.consoleLog("%s compiled by %s at %s (branch=%s, commit=%s)", 
		  		SendableVersion.INSTANCE.getProgramVersion(),
		  		SendableVersion.INSTANCE.getUser(),
				SendableVersion.INSTANCE.getTime(), 
				SendableVersion.INSTANCE.getBranch(),
				SendableVersion.INSTANCE.getCommit());

		  // Send program version to the dashboard.
	   		
		  SmartDashboard.putString("Program", PROGRAM_NAME);
		
		  // Log RobotLib version we are using.
		  Util.consoleLog("RobotLib=%s", LibraryVersion.version);
		  
		  // Note: Any Sendables added to SmartDashboard or Shuffleboard are sent to the DS on every
		  // loop of a TimedRobot. In this case it means that the SendableVersion data would be sent
		  // to the DS every 20ms even though it does not change. Sendables must be added to the SDB
		  // or SB in order to be sent so its a catch-22 with Sendables. So we add the SendableVersion
		  // here and then just below delete it from the sendable system. This puts the version info
		  // onto the dashboard but removes it from further updates.
	   		
		  SmartDashboard.putData("Version", SendableVersion.INSTANCE);
		  
		  // Instantiate our RobotContainer. This will perform all our button bindings, and put our
		  // autonomous chooser on the dashboard.
		  
		  robotContainer = new RobotContainer();
		  
		  SendableVersion.INSTANCE.removeSendable();
	  }
	  catch (Exception e) {Util.logException(e);}
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
	  // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
	  // commands, running already-scheduled commands, removing finished or interrupted commands,
	  // and running subsystem periodic() methods. This must be called from the robot's periodic
	  // function in order for anything in the Command-based framework to work.
	  
	  // WARNING: This function is called repeatedly even when robot is DISABLED. This means the
	  // periodic method in all subsystems will be called even when disabled. The scheduler will
	  // stop commands when disabled but not subsystems. It is possible to set Commands to run
	  // when robot is disabled. This seems a bad idea...
	  
	  CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() 
  {
	  Util.consoleLog();
      
	  LCD.printLine(LCD_1, "Mode: Disabled");
	  
	  // Reset driver station LEDs.

	  SmartDashboard.putBoolean("Disabled", true);
	  SmartDashboard.putBoolean("Auto Mode", false);
	  SmartDashboard.putBoolean("Teleop Mode", false);
	  SmartDashboard.putBoolean("FMS", ds.isFMSAttached());
	  SmartDashboard.putBoolean("Overload", false);
	  SmartDashboard.putNumber("AirPressure", 0);
	  SmartDashboard.putBoolean("AltDriveMode", false);
	  SmartDashboard.putBoolean("SteeringAssist", false);
	  SmartDashboard.putBoolean("Brake", false);
	  SmartDashboard.putBoolean("Pickup", false);
	  SmartDashboard.putBoolean("PickupExtended", false);
	  SmartDashboard.putBoolean("CountingTurns", false);
	  SmartDashboard.putBoolean("RotatingToTarget", false);
	  SmartDashboard.putBoolean("Shooter", false);
	  SmartDashboard.putBoolean("Belt", false);
	  SmartDashboard.putString("GameColor", "");

	  Util.consoleLog("end -------------------------------------------------------------------------");
  }

  /**
   * This function is called periodically during disabled mode.
   * Technically there should be nothing here.
   */
  @Override
  public void disabledPeriodic() 
  {
  }

  /**
   * This function is called once at the start of autonomous mode and
   * schedules the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() 
  {
	  Util.consoleLog();
	  
	  LCD.clearAll();
      
	  LCD.printLine(LCD_1, "Mode: Auto - No Program");

	  SmartDashboard.putBoolean("Disabled", false);
	  SmartDashboard.putBoolean("Auto Mode", true);
	  
	  robotContainer.getMatchInformation();
	  
	  robotContainer.resetFaults();

	  // RobotContainer figures out which auto command is selected to run.
	  
	  Command autonomousCommand = robotContainer.getAutonomousCommand();

	  // schedule the autonomous command (example)
	  
	  if (autonomousCommand != null) autonomousCommand.schedule();
	  
	  Util.consoleLog("end -------------------------------------------------------------------------");
  }

  /**
   * This function is called periodically during autonomous.
   * Technically there should be nothing here.
   */
  @Override
  public void autonomousPeriodic() 
  {
  }

  /**
   * This function is called once at the start of teleop mode.
   */
  @Override
  public void teleopInit()
  {
	  Util.consoleLog();
	  
	  robotContainer.getMatchInformation();
      
	  LCD.clearAll();
	  
	  LCD.printLine(LCD_1, "Mode: teleop  All=%s, Start=%d, FMS=%b, msg=%s", alliance.name(), location, 
			  		ds.isFMSAttached(), gameMessage);

	  SmartDashboard.putBoolean("Disabled", false);
	  SmartDashboard.putBoolean("Teleop Mode", true);
	  
	  robotContainer.resetFaults();
	  
	  // Driving handled by DriveCommand which is default command for the DriveBase.
	  // Other commands scheduled by joystick buttons.
	  
	  Util.consoleLog("end -------------------------------------------------------------------------");
  }

  /**
   * This function is called periodically during teleop.
   * Technically there should be nothing here. 2020 game has color wheel
   * target color that can be sent by FMS at any time so we monitor for
   * it here.
   */
  @Override
  public void teleopPeriodic() 
  {
		// Update game color on DS. Can change at any time during teleop.

		String gameData = ds.getGameSpecificMessage();
		
		if (gameData != null) SmartDashboard.putString("GameColor", ColorWheel.convertGameColor(gameData));
  }

  /**
   * This function is called once at the start of test mode.
   */
  @Override
  public void testInit() 
  {
	  Util.consoleLog();
	  
	  LCD.clearAll();
	  
	  // Cancels all running commands at the start of test mode.
	  
	  CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {
  }
}
