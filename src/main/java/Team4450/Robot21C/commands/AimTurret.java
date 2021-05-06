
package Team4450.Robot21C.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot21C.subsystems.Turret;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command that feeds utility stick X axis % power to the Turrent rotation motor.
 */
public class AimTurret extends CommandBase 
{
  private final Turret 		    turret;
    
  private final DoubleSupplier	rotationPower;
  private final BooleanSupplier shooterIsRunning;
  
  private boolean				endAiming;

  /**
   * Creates a new Turret Aiming command. Uses Utility stick X deflection
   * to move the turret right/left. Runs as Turret default command.
   *
   * @param subsystem The subsystem used by this command.
   * @param rotatePower A double supplier of the speed of rotation
   * as % power -1.0 to +1.0. Note, power is fixed in Turret class. Only
   * the sign is used to control direction, + right - left.
   * @param shooterIsRunning True if shooter wheel is running, false if not.
   */
  public AimTurret(Turret subsystem, DoubleSupplier rotationPower, BooleanSupplier shooterIsRunning) 
  {
	  Util.consoleLog();
	  
	  turret = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  
	  addRequirements(turret);
	  
      this.rotationPower = rotationPower;
      this.shooterIsRunning = shooterIsRunning;
  }

  /**
   *  Called when the command is initially scheduled.
   *  NOTE: This command is set as the default for the turret. That
   *  means it runs as long as no other command that uses the turret
   *  runs. If another command runs, this command will be interrupted 
   *  and then rescheduled when that other command is finished. If the
   *  robot is disabled and then reenabled, this command has been retained
   *  in memory and is rescheduled. This means this class persists in 
   *  memory as long as the robot is turned on. It will be interrupted
   *  and rescheduled as needed. Commands only cease to exist if they
   *  return true when thier isFinished() method is called by the 
   *  scheduler. You have to keep this in mind in terms of what reset
   *  is needed by this command when the robot is reenabled.
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
      // Squaring tones down the responsiveness of the turret.
      
	  //if (shooterIsRunning.getAsBoolean()) turret.rotate(Util.squareInput(rotationPower.getAsDouble()));
	  turret.rotate(Util.squareInput(rotationPower.getAsDouble()));
  }

  /**
   *  Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
	    Util.consoleLog("interrupted=%b", interrupted);
	
	    turret.stop();
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
   * End Aiming command.
   */
  public void stop()
  {
	  Util.consoleLog();
	  
	  endAiming = true;
  }
}

