package Team4450.Robot21C.commands;

import Team4450.Lib.Util;
import Team4450.Robot21C.subsystems.ColorWheel;
import static Team4450.Robot21C.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that turns the color wheel until target color is under the
 * field's color sensor.
 */
public class TurnWheelToColor extends CommandBase 
{
  private final ColorWheel	colorWheel;

  /**
   * Creates a new TurnWheelToColor command.
   * @param subsystem The subsystem used by this command.
   */
  public TurnWheelToColor(ColorWheel subsystem) 
  {
	  Util.consoleLog();
	  
	  colorWheel = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  addRequirements(colorWheel);
  }

  /** 
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() 
  {
	  Util.consoleLog();

	  SmartDashboard.putBoolean("RotatingToTarget", true);
		
	  // Set target color based on color sent by the FMS. Keep in
	  // mind the target color is the color that will be under robot
	  // color sensor when the FMS color is under the field sensor.
  	
	  colorWheel.setGameTargetColor();
	  
	  colorWheel.startWheel(COLORWHEEL_SPEED);
  }

  /**
   *  Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
	  Util.consoleLog("interrupted=%b", interrupted);
	  
	  SmartDashboard.putBoolean("RotatingToTarget", false);
	  
	  colorWheel.stopWheel();
  }

  /**
   *  Returns true when the command should end.
   */
  @Override
  public boolean isFinished() 
  {
	  // Stop when target color is detected by our sensor.

	  return colorWheel.isRunning() ? colorWheel.colorMatch(): true;
  }
}

