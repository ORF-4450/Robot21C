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
public class AutoRotateProfiled extends ProfiledPIDCommand 
{
    private DriveBase     driveBase;

    private static double kP = .2, kI = 0, kD = 0, toleranceDeg = .5, toleranceVelds = 10;
    private static double kMaxRotationVelds = 90, kMaxRotationAcceldss = 90;

    /**
     * Turns to robot to the specified angle using a motion profile.
     *
     * @param drive       The drive subsystem to use.
     * @param targetAngle The angle to turn to -180 to +180 degrees, + is clockwise.
     */
    public AutoRotateProfiled(DriveBase drive, double targetAngle) 
    {
        super(
            new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxRotationVelds, kMaxRotationAcceldss)),
            // Closed loop on yaw via reference so pid controller can call it on each execute() call.
            RobotContainer.navx::getYaw,
            // Set target angle
            targetAngle,
            // Pipe output to turn robot
            (output, setpoint) -> drive.arcadeDrive(0, output, false),
            // Require the drive
            drive);

        Util.checkRange(targetAngle, 180, "target angle");

        Util.consoleLog("angle=%.2f  kP=%.3f  kI=%.3f", targetAngle, kP, kI);

        driveBase = drive;

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);

        // Set the controller tolerance - the velocity tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(toleranceDeg, toleranceVelds);
    }
        
    @Override
    public void initialize()
    {
        Util.consoleLog();

        // Try to prevent over rotation.
        driveBase.SetCANTalonBrakeMode(true);
        
        // Reset gyro yaw to zero.
        RobotContainer.navx.resetYawWait(1, 1000);
    }

    @Override
    public void execute()
    {
        super.execute();

        Util.consoleLog("yaw=%.2f  hdng=%.2f  lpwr=%.2f  rpwr=%.2f", RobotContainer.navx.getYaw(), 
                        RobotContainer.navx.getHeading(), -driveBase.getLeftPower(), driveBase.getRightPower());
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
                        RobotContainer.navx.getYaw());

		// Wait for robot to stop moving.
		Util.consoleLog("moving=%b", RobotContainer.navx.isRotating());
		
		//while (RobotContainer.navx.isRotating()) {Timer.delay(.10);}
		//Util.consoleLog("moving=%b", Devices.navx.isRotating());
		
		Util.consoleLog("2  hdg=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), RobotContainer.navx.getYaw());
		Util.consoleLog("end -------------------------------------------------------------------------");
	}
}