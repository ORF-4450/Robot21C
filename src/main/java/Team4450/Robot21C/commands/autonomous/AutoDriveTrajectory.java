package Team4450.Robot21C.commands.autonomous;

import Team4450.Robot21C.commands.autonomous.AutoDrive.Brakes;
import Team4450.Robot21C.commands.autonomous.AutoDrive.StopMotors;
import Team4450.Robot21C.subsystems.DriveBase;
import Team4450.Lib.Util;
import static Team4450.Robot21C.Constants.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * A command that will follow a trajectory using a Ramsete controller. All of the parameters
 * below are a guess. Need to characterize the robot to get good numbers. Works so-so with
 * the guesses.
 */
public class AutoDriveTrajectory extends RamseteCommand
{
    private static double   kP = .137, kI = kP / 100, kD = 0, startTime;
    private int             iterations;

    private DriveBase       driveBase;
    private Trajectory      trajectory;
    private StopMotors      stop;
    private Brakes          brakes;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    private static double   kRamseteB = 2, kRamseteZeta = .7;
 
    /**
     * Auto drive the given trajectory.
     * @param driveBase     Drive base to use.
     * @param trajectory    Trajectory to follow.
     * @param stop          Set stop or not at trajectory end.
     * @param brakes        If stopping, set if brakes on or off.
     */
    AutoDriveTrajectory(DriveBase driveBase, Trajectory trajectory, StopMotors stop, Brakes brakes)
    {
        super(trajectory,
                driveBase::getOdometerPose,
                new RamseteController(kRamseteB, kRamseteZeta),
                new SimpleMotorFeedforward(DB_KS, DB_KV, DB_KA),
                new DifferentialDriveKinematics(Util.inchesToMeters(TRACK_WIDTH)),
                //new DifferentialDriveKinematics(TRACK_WIDTH_C),
                driveBase::getWheelSpeeds,
                new PIDController(kP, kI, kD),  // left motor PID controller.
                new PIDController(kP, kI, kD),  // Right motor PID controller.
                // RamseteCommand passes left/right volts to the drive base callback.
                driveBase::setVoltage2,
                driveBase);

        this.driveBase = driveBase;
        this.trajectory = trajectory;
        this.stop = stop;
        this.brakes = brakes;
    }
    
    @Override
    public void initialize()
    {
        Util.consoleLog();

        startTime = Util.timeStamp();

        super.initialize();

        if (brakes == Brakes.on)
            driveBase.SetCANTalonBrakeMode(true);
        else
            driveBase.SetCANTalonBrakeMode(false);

        // Set the current robot pose to match the starting pose of the trajectory. If all of your
        // autonomouse moves are correctly coordinated the starting pose of the trajectory should
        // match the physical pose of the robot.
        
        Pose2d pose = trajectory.getInitialPose();

        Util.consoleLog("initial traj poseX=%.2f  poseY=%.2f  poseHdg=%.2f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
        
        driveBase.resetOdometer(pose, pose.getRotation().getDegrees());

        //pose = driveBase.getOdometerPose();

        //Util.consoleLog("initial robot poseX=%.2f  poseY=%.2f  poseHdg=%.2f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    @Override
    public void execute()
    {
        Util.consoleLog();

        super.execute();
        
        Pose2d pose = driveBase.getOdometerPose();

        Util.consoleLog("robot poseX=%.2f  poseY=%.2f  poseAng=%.2f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());

        iterations++;
    }
    
    @Override
    public boolean isFinished() 
    {
        // End when the controller has finished the trajectory.
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
        
        super.end(interrupted);

        if (stop == StopMotors.stop) driveBase.stop();
                
        Pose2d pose = driveBase.getOdometerPose();

        Util.consoleLog("poseX=%.2f  poseY=%.2f  poseAng=%.2f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());

		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

        Util.consoleLog("end ----------------------------------------------------------");
    }

    /**
     * Generate a VoltageConstraint for use in generating a TrajectoryConfig. Uses track width
     * and a feed forward based on characterization of the drive base.
     * @return The VoltageConstraint.
     */
    public static DifferentialDriveVoltageConstraint getVoltageConstraint()
    {
        return new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(DB_KS, DB_KV, DB_KA),
                    new DifferentialDriveKinematics(Util.inchesToMeters(TRACK_WIDTH)),
                    //new DifferentialDriveKinematics(TRACK_WIDTH_C),
                    10);    // 10v to leave some room to go over the constraint.
    }

    /**
     * Generate a TrajectoryConfig given a VoltageConstraint and wheel speed info.
     * @return The TrajectoryConfig.
     */
    public static TrajectoryConfig getTrajectoryConfig(DifferentialDriveVoltageConstraint voltageConstraint)
    {
        return new TrajectoryConfig(MAX_WHEEL_SPEED, MAX_WHEEL_ACCEL)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(new DifferentialDriveKinematics(Util.inchesToMeters(TRACK_WIDTH)))
                    //.setKinematics(new DifferentialDriveKinematics(TRACK_WIDTH_C))
                    // Apply the voltage constraint
                    .addConstraint(voltageConstraint);
    }
}
