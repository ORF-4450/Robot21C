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

  private static double kP = .005, kI = .01, kD = 0, kToleranceDeg = 1, kToleranceVelds = 10;
  private static double kMaxRotationVelds = 90, kMaxRotationAcceldss = 90, startTime;
  private int           iterations;
  
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param drive         The drive subsystem to use.
   * @param heading       The heading to turn to 0 to 359 degrees.
   */
  public AutoRotateHdgProfiled(DriveBase drive, double heading) 
  {
    super(
      new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxRotationVelds, kMaxRotationAcceldss)),
      // Closed loop on heading via reference so pid controller can call it on each execute() call.
      RobotContainer.navx::getHeading,
      // Set target heading.
      heading,
      // Pipe output to turn robot
      (output, setpoint) -> drive.arcadeDrive(0, output, false),
      // Require the drive
      drive);

    Util.checkRange(heading, 0, 359, "heading");

    Util.consoleLog("heading=%.1f  kP=%.3f  kI=%.3f", heading, kP, kI);

    driveBase = drive;
    
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);

    // Set the controller tolerance - the velocity tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(kToleranceDeg);//, kToleranceVelds);
  }
  
  @Override
  public void initialize()
  {
    Util.consoleLog();

    startTime = Util.timeStamp();

    // Try to prevent over rotation.
    driveBase.SetCANTalonBrakeMode(true);

    // Do not reset navx in this routine. It will foul up first call to getHeading().

    Util.consoleLog("start hdng=%.2f", RobotContainer.navx.getHeading());

    // Set profile controller initial position.
    getController().reset(RobotContainer.navx.getHeading());
  }

  @Override
  public void execute()
  {
      super.execute();

      Util.consoleLog("yaw=%.2f hdngyaw=%.2f hdng=%.2f lpwr=%.2f rpwr=%.2f", RobotContainer.navx.getYaw(), 
                      RobotContainer.navx.getHeadingYaw(), RobotContainer.navx.getHeading(), 
                      driveBase.getLeftPower(), -driveBase.getRightPower());

      //Util.consoleLog("goal=%.2f  sp=%.5f  m=%.3f  err=%.3f", getController().getGoal().position,
      //                getController().getSetpoint().position, m_measurement.getAsDouble(),
      //                getController().getPositionError());

      iterations++;
  }


  @Override
  public boolean isFinished() 
  {
      // End when the controller is at the reference.
      return getController().atGoal();
  }
      
  @Override
  public void end(boolean interrupted) 
  {
    Util.consoleLog("interrupted=%b", interrupted);

    driveBase.stop();

    Util.consoleLog("after stop  hdg=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), 
                        RobotContainer.navx.getHeadingYaw());

    // Wait for robot to stop moving.
    Util.consoleLog("moving=%b", RobotContainer.navx.isRotating());

    //while (RobotContainer.navx.isRotating()) {Timer.delay(.10);}
    //Util.consoleLog("moving=%b", Devices.navx.isRotating());

    Util.consoleLog("2  hdg=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), 
                    RobotContainer.navx.getHeadingYaw());

    Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

    Util.consoleLog("end -----------------------------------------------------");
  }
}