package Team4450.Robot21C.commands;

import Team4450.Robot21C.subsystems.DriveBase;
import Team4450.Lib.Util;
import Team4450.Robot21C.RobotContainer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/**
 * A command that will turn the robot to the specified angle using a motion profile.
 */
public class AutoRotateProfiled extends ProfiledPIDCommand 
{
    private DriveBase    driveBase;

    private static AutoRotateProfiled    thisInstance;

    private static double kP = .1, kI = .15, kD = 0, toleranceDeg = .5, toleranceVelds = 1;
    private static double kMaxRotationVelds = 70, kMaxRotationAcceldss = 70;
    private double        targetAngle, startTime;
    private int           iterations;

    // Estimate feed forward gains as 12v / max velocity.
    private SimpleMotorFeedforward  feedForward = new SimpleMotorFeedforward(12 / kMaxRotationVelds, 
                                                                             12 / kMaxRotationVelds);

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
            (output, setpoint) -> thisInstance.driveWithFeedForward(output, setpoint),
            // Require the drive base
            drive);

        Util.checkRange(targetAngle, 180, "target angle");

        Util.consoleLog("angle=%.2f  kP=%.3f  kI=%.3f", targetAngle, kP, kI);

        driveBase = drive;
        this.targetAngle = targetAngle;
        thisInstance = this;

        // Set the controller to be continuous (because it is an angle controller)
        //getController().enableContinuousInput(-180, 180);

        // Set the controller tolerance - the velocity tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(toleranceDeg); //, toleranceVelds);
    }

    // Drive combining feed forward with PID output (angle error). The set point computed
    // by the motion profile feeds the feed forward calculation. So the profile drives both
    // the feed forward (base power setting in volts) and we add in a factor from the PID
    // based on the error between angle setpoint and measured angle.
    private void driveWithFeedForward(double power, TrapezoidProfile.State setPoint)
    {
        double ff = feedForward.calculate(setPoint.velocity);

        Util.consoleLog("ff=%.3fv", ff);

        // Feed forward is in volts so we convert the PID output (degrees) to
        // voltage so it combines with ff and then we set motors with voltage.
        power = power * 12 + ff;

        driveBase.setVoltage(power, power);
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

        Util.consoleLog("goal=%.2f  sp=%.3f  m=%.3f  err=%.3f", getController().getGoal().position,
                        getController().getSetpoint().position, m_measurement.getAsDouble(),
                        getController().getPositionError());

        Util.consoleLog("yaw=%.3f  hdng=%.2f  lpwr=%.2f  rpwr=%.2f", RobotContainer.navx.getYaw(),
                        RobotContainer.navx.getHeading(), driveBase.getLeftPower(), -driveBase.getRightPower());

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