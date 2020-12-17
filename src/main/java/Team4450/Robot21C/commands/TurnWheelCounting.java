package Team4450.Robot21C.commands;

import Team4450.Lib.Util;
import Team4450.Robot21C.subsystems.ColorWheel;
import static Team4450.Robot21C.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that turns the color wheel counting the number of turns
 * until target number of turns are completed.
 */
public class TurnWheelCounting extends CommandBase 
{
  private final ColorWheel	colorWheel;
  private int				turnCount;
  private boolean			onTargetColor;

  /**
   * Creates a new TurnWheelCounting command.
   * @param subsystem The subsystem used by this command.
   */
  public TurnWheelCounting(ColorWheel subsystem) 
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

	  SmartDashboard.putBoolean("CountingTurns", true);
		
	  // Current color is color we will count when it passes by the sensor
	  // after the first time. We have to detect color change and count the
	  // change if it changes back to target color from not target color.
  	
	  colorWheel.setTargetToCurrentColor();
		
	  turnCount = 0;
	  
	  onTargetColor = true;
	  
	  colorWheel.startWheel(COLORWHEEL_SPEED);
  }

  /** 
   * Called every time the scheduler runs while the command is scheduled.
   * This repeatedly checks the color sensor while the wheel is turning
   * and counts color changes
   */
  @Override
  public void execute() 
  {
		if (colorWheel.colorMatch())
		{
			if (!onTargetColor)
			{
				onTargetColor = true;
				turnCount++;
			}
		} else onTargetColor = false;
  }

  /**
   *  Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
	  Util.consoleLog("interrupted=%b", interrupted);
	  
	  SmartDashboard.putBoolean("CountingTurns", false);
	  
	  colorWheel.stopWheel();
  }

  /**
   *  Returns true when the command should end.
   */
  @Override
  public boolean isFinished() 
  {
	  // Count 2x color changes since each color appears twice on wheel.

	  return colorWheel.isRunning() ? turnCount > COLORWHEEL_ROTATIONS * 2 : true;
  }
}

