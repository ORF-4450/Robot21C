
package Team4450.Robot21C.commands;

import java.util.function.DoubleSupplier;

import static Team4450.Robot21C.Constants.*;
import Team4450.Robot21C.RobotContainer;
import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Robot21C.subsystems.DriveBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Driving command that feeds target speed and rotation (% power) to the DriveBase.
 * Used for single stick arcade style driving.
 */
public class ArcadeDrive extends CommandBase 
{
  private final DriveBase 		driveBase;
  
  private final DoubleSupplier	speedSupplier, rotationSupplier;

  private final double        steeringGain = .7;
  
  /**
   * Creates a new ArcadeDrive command.
   *
   * @param subsystem The subsystem used by this command.
   * @param speed The speed as % power -1.0 to +1.0. + is forward.
   * @param rotation The rotation as % power -1.0 to +1.0. + is clockwise.
   */
  public ArcadeDrive(DriveBase subsystem, DoubleSupplier speed, DoubleSupplier rotation) 
  {
      Util.consoleLog();
	  
	  driveBase = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  
	  addRequirements(this.driveBase);
	  
	  // Save references to DoubleSupplier objects so we can read them later in the
	  // execute method.
	  
	  speedSupplier = speed;
	  rotationSupplier = rotation;
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
   *  then execute resumes being repeatedly called. All commands work like this.
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
   * the speed & rotation values provided by whatever double provider was passed
   * in the constructor to the drive base arcade drive function. The providers are
   * typically the joystick X & Y deflection values but can be any double provider.
   */
  @Override
  public void execute() 
  {
	  double speed = speedSupplier.getAsDouble(), rotation = rotationSupplier.getAsDouble();
	  
	  LCD.printLine(LCD_2, "leftenc=%d  rightenc=%d", driveBase.getLeftEncoder(), driveBase.getRightEncoder());			

      LCD.printLine(LCD_3, "speed=%.3f  rotation=%.3f  (lpwr=%.3f) (rpwr=%.3f)", speed, rotation, 
                driveBase.getLeftPower(), driveBase.getRightPower());

	  LCD.printLine(LCD_7, "Lrpm=%d - Rrpm=%d  Lmax vel=%.3f - Rmax vel=%.3f", driveBase.leftEncoder.getRPM(),
			  driveBase.rightEncoder.getRPM(), driveBase.leftEncoder.getMaxVelocity(PIDRateType.velocityMPS),
			  driveBase.rightEncoder.getMaxVelocity(PIDRateType.velocityMPS));
	  
	  Pose2d pose = driveBase.getOdometerPose();
	  
	  LCD.printLine(LCD_8, "pose x=%.1fm  y=%.1fm  deg=%.1f  balleye=%b ", pose.getTranslation().getX(), pose.getTranslation().getY(),
					pose.getRotation().getDegrees(), RobotContainer.pickup.getBallEye());
	  
	  driveBase.arcadeDrive(speed, rotation * steeringGain, true);
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

  /**
   * Method present to allow TankDrive and ArcadeDrive to be instantiated into the
   * same command. Does nothing on ArcadeDrive.
   */
  public void toggleAlternateDrivingMode()
  {
  }
}
