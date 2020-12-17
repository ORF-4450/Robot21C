package Team4450.Robot21C.commands;

import Team4450.Lib.Util;
import Team4450.Robot21C.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that toggles gear selection on the DriveBase.
 */
public class ShiftGears extends CommandBase 
{
  private final DriveBase driveBase;

  /**
   * Creates a new ShiftGears command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShiftGears(DriveBase subsystem) 
  {
	  Util.consoleLog();
	  
	  driveBase = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  addRequirements(driveBase);
  }

  /**
   * Called when the command is initially scheduled. Toggles high/low gear.
   */
  @Override
  public void initialize() 
  {
	  Util.consoleLog();
	  
	  if (driveBase.isLowSpeed())
		  driveBase.highSpeed();
	  else
		  driveBase.lowSpeed();
  }
  
  /** 
   * Returns true when the command should end. This means one execution\
   * accomplished in initialize function.
   */
  @Override
  public boolean isFinished() 
  {
	  return true;
  }
}
