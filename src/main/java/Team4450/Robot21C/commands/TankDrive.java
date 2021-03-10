
package Team4450.Robot21C.commands;

import java.util.function.DoubleSupplier;

import static Team4450.Robot21C.Constants.*;

import Team4450.Robot21C.Robot;
import Team4450.Robot21C.RobotContainer;
import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Robot21C.subsystems.DriveBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Driving command that feeds target left/right speeds (% power) to the DriveBase.
 * Used for two stick tank style driving.
 */
public class TankDrive extends CommandBase 
{
  private final DriveBase 		driveBase;
  
  private final DoubleSupplier	leftPower, rightPower;
  
  private boolean				altDriveMode, steeringAssistMode;
  private double                kPowerGain = .75;

  /**
   * Creates a new TankDrive command.
   *
   * @param subsystem The subsystem used by this command.
   * @param leftSpeed The speed as % power -1.0 to +1.0.
   * @param rightSpeed The speed as % power -1.0 to +1.0.
   */
  public TankDrive(DriveBase subsystem, DoubleSupplier leftPower, DoubleSupplier rightPower) 
  {
	  Util.consoleLog();
	  
	  driveBase = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  
	  addRequirements(this.driveBase);
	  
	  // Save references to DoubleSupplier objects so we can read them later in the
	  // execute method.
	  
	  this.leftPower = leftPower;
	  this.rightPower = rightPower;
  }

  /**
   *  Called when the command is scheduled to run.
   *  NOTE: This command is set as the default for the drive base. That
   *  means it runs as long as no other command that uses the drive base
   *  runs. If another command runs, like shift gears for instance, this
   *  command will be interrupted and then rescheduled when shift gears
   *  is finished. That reschedule means initialize() is called again.
   *  So it is important to realize that while this command class exists for
   *  the entire run of teleop, it stops when it is preempted by another
   *  command and then when rescheduled initialize will be called again and
   *  then execute resumes being repeatedly called. When the robot is disabled
   *  this command is interrupted but retained by the scheduler. When reenabled
   *  scheduler will call initialize to restart running this command. Commands
   *  only cease to exist (deleted by scheduler) when thier isFinished method
   *  returns true. All commands work like this.
   */
  @Override
  public void initialize() 
  {
	  Util.consoleLog();
	  
	  driveBase.setMotorSafety(true); 	// Turn on watchdog.

	  // 2018 post season testing showed this setting helps smooth out driving response.
	  // Set here because auto programs may set their own rate. We combine this with
	  // squared input on drive methods to try to reduce how jerky and touchy the 
	  // robot can be.
	  
	  driveBase.SetCANTalonRampRate(TALON_RAMP_RATE);
  }

  /** 
   * Called every time the scheduler runs while the command is scheduled. Passes
   * the left/right speed values provided by whatever double provider was passed
   * in the constructor to the drive base tank drive function. The providers are
   * typically the joystick Y deflection values but can be any double provider.
   */
  @Override
  public void execute() 
  {
	  double leftY = leftPower.getAsDouble() * kPowerGain, rightY = rightPower.getAsDouble() * kPowerGain, angle;
	  
	  LCD.printLine(LCD_2, "leftenc=%d  rightenc=%d", driveBase.getLeftEncoder(), driveBase.getRightEncoder());			

	  LCD.printLine(LCD_3, "leftY=%.3f (%.3f)  rightY=%.3f (%.3f)", leftY, 
                 driveBase.getLeftPower(), rightY, driveBase.getRightPower());
                 
      LCD.printLine(LCD_4, "utilY=%.3f  utilX=%.3f", RobotContainer.utilityStick.GetY(), RobotContainer.utilityStick.GetX());

	  LCD.printLine(LCD_7, "Lrpm=%d - Rrpm=%d  Lmax vel=%.3f - Rmax vel=%.3f", driveBase.leftEncoder.getRPM(),
			  driveBase.rightEncoder.getRPM(), driveBase.leftEncoder.getMaxVelocity(PIDRateType.velocityMPS),
			  driveBase.rightEncoder.getMaxVelocity(PIDRateType.velocityMPS));
	  
	  Pose2d pose = driveBase.getOdometerPose();
	  
      LCD.printLine(LCD_8, "pose x=%.1fm (lrot=%.2f)  y=%.1fm  deg=%.1f  balleye=%b ", pose.getX(), 
                    driveBase.leftEncoder.getRotations(), pose.getY(),
                    pose.getRotation().getDegrees(), RobotContainer.pickup.getBallEye());
                    
      LCD.printLine(LCD_9, "shooter rpm=%.0f  max=%.0f", RobotContainer.shooter.getRPM(), 
                    RobotContainer.shooter.getMaxRPM());
	  
	  if (altDriveMode)
	  {	  // normal tank with straight drive assist when sticks within 10% of each other and
		  // right stick power is greater than 50%.
		  if (isLeftRightEqual(leftY, rightY, 10) && Math.abs(rightY) > .50)
		  {
			  // Reset angle measurement when entering this code first time after mode is enabled.
			  if (!steeringAssistMode) RobotContainer.navx.resetYaw();
		
			  // Angle is negative if robot veering left, positive if veering right when going forward.
			  // It is opposite when going backward.
		
			  angle = (int) RobotContainer.navx.getYaw();
		
			  //LCD.printLine(5, "angle=%d", angle);
		
			  // Invert angle for backwards movement.
		
			  if (rightY < 0) angle = -angle;
		
			  //Util.consoleLog("angle=%d", angle);
		
			  // Note we invert sign on the angle because we want the robot to turn in the opposite
			  // direction than it is currently going to correct it. So a + angle says robot is veering
			  // right so we set the turn value to - because - is a turn left which corrects our right
			  // drift.
				
			  driveBase.curvatureDrive(rightY, -angle * STEERING_ASSIST_GAIN, false);
		
			  steeringAssistMode = true;
		  }
		  else
		  {
			  steeringAssistMode = false;
			  driveBase.tankDrive(leftY, rightY, true);	// Normal tank drive.
		  }
	
		  SmartDashboard.putBoolean("SteeringAssist", steeringAssistMode);
	  }
	  else
		  driveBase.tankDrive(leftY, rightY, true);		// Normal tank drive.
  }

  private boolean isLeftRightEqual(double left, double right, double percent)
  {
	  if (Math.abs(left - right) <= (1 * (percent / 100))) return true;

	  return false;
  }
  
  /**
   * Toggles alternate drive mode.
   */
  public void toggleAlternateDrivingMode()
  {
	  Util.consoleLog("%b", altDriveMode);
	  
	  altDriveMode = !altDriveMode;
	  
	  steeringAssistMode = false;
	  
	  SmartDashboard.putBoolean("AltDriveMode", altDriveMode);
  }
  
  /**
   *  Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
	  Util.consoleLog("interrupted=%b", interrupted);
	  
	  driveBase.stop();
	  
	  driveBase.setMotorSafety(false); 	// Turn off watchdog.
  }

  /**
   *  Returns true when the command should end. Returning false means it never ends.
   */
  @Override
  public boolean isFinished() 
  {
	  return false;
  }
}
