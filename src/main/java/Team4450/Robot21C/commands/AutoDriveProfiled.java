package Team4450.Robot21C.commands;

import Team4450.Robot21C.subsystems.DriveBase;
import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import static Team4450.Robot21C.Constants.*;

import Team4450.Robot21C.RobotContainer;
import Team4450.Robot21C.commands.AutoDrive.Brakes;
import Team4450.Robot21C.commands.AutoDrive.StopMotors;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/**
 * A command that will drive the robot to the specified distance using a motion profile and
 * steering correction.
 */
public class AutoDriveProfiled extends ProfiledPIDCommand 
{
    private DriveBase     driveBase;

    private static double kP = .75, kI = 0, kD = 0, toleranceMeters = .05;
    private static double kMaxVelms = 3, kMaxAccelmss = 10, curve;
    private double        distance, kSteeringGain = .07;
    private StopMotors    stop;
    private Brakes        brakes;
    
    /**
     * Drives robot to the specified distance using a motion profile with straight steering.
     *
     * @param drive         The drive subsystem to use.
     * @param distance      The distance to drive in meters. + is forward.
     * @param stop          Set to stop or not stop motors at end.
     * @param brakes        If stopping, set brakes on or off.
     */
    public AutoDriveProfiled(DriveBase drive, double distance, StopMotors stop, Brakes brakes) 
    {
        super(
            new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxVelms, kMaxAccelmss)),
            // Closed loop on encoder distance via reference so pid controller can call it on each execute() call.
            drive::getAvgEncoderDist,
            // Set target distance.
            (double) distance,
            // Pipe output to drive robot
            (output, setpoint) -> drive.arcadeDrive(output, curve, false),
            // Require the drive
            drive);

        Util.consoleLog("distance=%.3fm  stop=%s  brakes=%s", distance, stop, brakes);
            
        Util.consoleLog("kP=%.6f  kI=%.6f", kP, kI);

        driveBase = drive;
        this.brakes = brakes;
        this.stop = stop;
        this.distance = distance;

        // Set the controller tolerance.
        getController().setTolerance(toleranceMeters);
    }
        
    @Override
    public void initialize()
    {
        Util.consoleLog();

        if (brakes == Brakes.on)
            driveBase.SetCANTalonBrakeMode(true);
        else
            driveBase.SetCANTalonBrakeMode(false);
        
        driveBase.resetEncodersWithDelay();
        
        RobotContainer.navx.resetYaw();
    }
    
    public void execute() 
    {
        Util.consoleLog();
        
        double yaw = RobotContainer.navx.getYaw();

        curve = Util.clampValue(-yaw * kSteeringGain, 1.0);

        super.execute();

        LCD.printLine(LCD_4, "Auto wheel encoder avg=%d  dist=%.3f", driveBase.getAvgEncoder(), 
                      driveBase.getAvgEncoderDist());

        Util.consoleLog("target=%.3f  avgdist=%.3f  error=%.3f  yaw=%.2f  curve=%.2f", getController().getGoal().position, 
                        driveBase.getAvgEncoderDist(), getController().getPositionError(), yaw, curve);
    }

    @Override
    public boolean isFinished() 
    {
        // End when the controller is at the goal.
        return getController().atGoal();
    }
	
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
		if (stop == StopMotors.stop) driveBase.stop();
		
		double actualDist = Math.abs(driveBase.getAvgEncoderDist());
		
		Util.consoleLog("end: target=%.3f  actual=%.3f  error=%.2f pct  hdng=%.2f", Math.abs(distance), actualDist, 
                (actualDist - Math.abs(distance)) / Math.abs(distance) * 100.0, RobotContainer.navx.getHeading());
                
		Util.consoleLog("end -------------------------------------------------------------------------");
	}
}