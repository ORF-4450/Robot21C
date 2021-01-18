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

    private static double kP = 1, kI = .02, kD = 0, toleranceDeg = .5, toleranceVelds = 1;
    private static double kMaxRotationVelds = 270, kMaxRotationAcceldss = 270;
    private double        targetAngle, startTime;
    private int           iterations;

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
            (output, setpoint) -> drive.curvatureDrive(0, output, true),
            //(output, setpoint) -> drive.arcadeDrive(0, output, false),
            // Require the drive base
            drive);

        Util.checkRange(targetAngle, 180, "target angle");

        Util.consoleLog("angle=%.2f  kP=%.3f  kI=%.3f", targetAngle, kP, kI);

        driveBase = drive;
        this.targetAngle = targetAngle;

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);

        // Set the controller tolerance - the velocity tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(toleranceDeg); //, toleranceVelds);
    }
        
    @Override
    public void initialize()
    {
        Util.consoleLog();

        startTime = Util.timeStamp();

        // Try to prevent over rotation.
        driveBase.SetCANTalonBrakeMode(true);
        
        // Reset gyro yaw to zero, wait up to 5 sec for navx to return zero yaw.
        RobotContainer.navx.resetYawWait(1, 5000);
    }

    @Override
    public void execute()
    {
        super.execute();

        Util.consoleLog("target=%.2f  yaw=%.3f  hdng=%.2f  lpwr=%.2f  rpwr=%.2f", targetAngle,
                        RobotContainer.navx.getYaw(), RobotContainer.navx.getHeading(),
                        driveBase.getLeftPower(), -driveBase.getRightPower());

        Util.consoleLog("goal=%.2f  sp=%.3f  m=%.3f  err=%.3f", getController().getGoal().position,
                        getController().getSetpoint().position, m_measurement.getAsDouble(),
                        getController().getPositionError());

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
                        RobotContainer.navx.getYaw());

		// Wait for robot to stop moving.
		Util.consoleLog("moving=%b", RobotContainer.navx.isRotating());
		
		//while (RobotContainer.navx.isRotating()) {Timer.delay(.10);}
		//Util.consoleLog("moving=%b", Devices.navx.isRotating());
		
		Util.consoleLog("2  hdg=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), RobotContainer.navx.getYaw());
        
		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

        Util.consoleLog("end ----------------------------------------------------------");
	}
}