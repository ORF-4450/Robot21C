package Team4450.Robot21C.commands;

import Team4450.Robot21C.subsystems.DriveBase;
import Team4450.Lib.Util;
import Team4450.Robot21C.RobotContainer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/**
 * A command that will turn the robot to the specified angle using a motion profile.
 */
public class AutoRotateHdgProfiled extends ProfiledPIDCommand 
{
  private DriveBase     driveBase;

  private static double kP = .1, kI = 0, kD = 0, toleranceDeg = .4, toleranceVel = 10;
  private static double kMaxRotationVel = 90, kMaxRotationAccel = 90;
  private double        targetHeading;
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetHeading The heading to turn to 0 to 359 degrees.
   * @param drive         The drive subsystem to use.
   */
  public AutoRotateHdgProfiled(DriveBase drive, double targetHeading) 
  {
    super(
        new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxRotationVel, kMaxRotationAccel)),
        // Closed loop on heading yaw
        RobotContainer.navx::getHeading,
        // Set reference to target. Here we are trying to get to zero yaw from target heading set below.
        targetHeading,
        // Pipe output to turn robot
        (output, setpoint) -> drive.arcadeDrive(0, output, false),
        // Require the drive
        drive);

    Util.checkRange(targetHeading, 0, 359, "target heading");

    this.targetHeading = targetHeading;

    Util.consoleLog("heading= %.1f  kP=%.3f  kI=%.3f", targetHeading, kP, kI);

    driveBase = drive;

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(toleranceDeg, toleranceVel);
  }
	
  @Override
  public void initialize()
  {
      Util.consoleLog();

      // Try to prevent over rotation.
      driveBase.SetCANTalonBrakeMode(true);
    
      // Set gyro target heading.
      //RobotContainer.navx.setTargetHeading(targetHeading);

      m_controller.reset(RobotContainer.navx.getHeading());
  }
  
  @Override
  public boolean isFinished() 
  {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}