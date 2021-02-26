
package Team4450.Robot21C.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot21C.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that feeds % power to the Turrent rotation motor.
 */
public class AimTurret extends CommandBase 
{
  private final Turret 		    turret;
    
  private final DoubleSupplier	rotationPower;
  
  private boolean				endAiming;

  /**
   * Creates a new Traverse command.
   *
   * @param subsystem The subsystem used by this command.
   * @param rotatePower A double supplier of the speed of rotation
   * as % power -1.0 to +1.0. Note, power is fixed in Turret class. Only
   * the sign is used to control direction, + right - left.
   */
  public AimTurret(Turret subsystem, DoubleSupplier rotationPower) 
  {
	  Util.consoleLog();
	  
	  turret = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  
	  addRequirements(this.turret);
	  
	  this.rotationPower = rotationPower;
  }

  /**
   *  Called when the command is initially scheduled.
   *  NOTE: This command is set as the default for the turret. That
   *  means it runs as long as no other command that uses the turret
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
	  
	  endAiming = false;
  }

  /** 
   * Called every time the scheduler runs while the command is scheduled. Passes
   * the rotation power value provided by whatever double provider was passed
   * in the constructor to the turret rotate() function. The provider is
   * typically the utility stick X deflection value but can be any double provider.
   */
  @Override
  public void execute() 
  {
	  // Squaring tones down the responsiveness of the winch.
	  turret.rotate(Util.squareInput(rotationPower.getAsDouble()));
  }

  /**
   *  Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
	    Util.consoleLog("interrupted=%b", interrupted);
	  
	    turret.rotate(0);
  }

  /**
   *  Returns true when the command should end. Returning false means it never ends.
   */
  @Override
  public boolean isFinished() 
  {
	  return endAiming;
  }
  
  /**
   * End Climber traverse mode.
   */
  public void stop()
  {
	  Util.consoleLog();
	  
	  endAiming = true;
  }
}

