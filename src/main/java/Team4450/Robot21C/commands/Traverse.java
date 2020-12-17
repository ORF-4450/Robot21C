
package Team4450.Robot21C.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot21C.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that feeds % power to the Climber traverse motor.
 */
public class Traverse extends CommandBase 
{
  private final Climber 		climber;
    
  private final DoubleSupplier	traversePower;
  
  private boolean				endTraverse;

  /**
   * Creates a new Traverse command.
   *
   * @param subsystem The subsystem used by this command.
   * @param traversePower A double supplier of the speed of traverse
   * as % power -1.0 to +1.0.
   */
  public Traverse(Climber subsystem, DoubleSupplier traversePower) 
  {
	  Util.consoleLog();
	  
	  climber = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  
	  addRequirements(this.climber);
	  
	  this.traversePower = traversePower;
  }

  /**
   *  Called when the command is initially scheduled.
   *  NOTE: This command is set as the default for the climber. That
   *  means it runs as long as no other command that uses the climber
   *  runs. If another command runs, this command will be interrupted 
   *  and then rescheduled when that other command is finished. That 
   *  reschedule means initialize() is called again. So it is important 
   *  to realize this command does not "exist" for the entire run of teleop.
   *  It comes and goes when it is preempted by another command. 
   *  All commands work like this.
   */
  @Override
  public void initialize() 
  {
	  Util.consoleLog();
	  
	  endTraverse = false;
  }

  /** 
   * Called every time the scheduler runs while the command is scheduled. Passes
   * the traverse power value provided by whatever double provider was passed
   * in the constructor to the climber setTraversePower() function. The provider is
   * typically the utility stick X deflection value but can be any double provider.
   */
  @Override
  public void execute() 
  {
	  // Squaring tones down the responsiveness of the winch.
	  climber.setTraversePower(Util.squareInput(traversePower.getAsDouble()));
  }

  /**
   *  Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
	  Util.consoleLog("interrupted=%b", interrupted);
	  
	  climber.stop();
  }

  /**
   *  Returns true when the command should end. Returning false means it never ends.
   */
  @Override
  public boolean isFinished() 
  {
	  return endTraverse;
  }
  
  /**
   * End Climber traverse mode.
   */
  public void stop()
  {
	  Util.consoleLog();
	  
	  endTraverse = true;
  }
}

