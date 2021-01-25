package Team4450.Robot21C.commands;

import Team4450.Robot21C.subsystems.DriveBase;
import Team4450.Lib.Util;
import static Team4450.Robot21C.Constants.*;
import Team4450.Robot21C.RobotContainer;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class AutoDriveTrajectory extends RamseteCommand
{
    private static double   kP = 12 / MAX_WHEEL_SPEED_MS, kI = kP / 100, kD = 0, startTime;
    private int             iterations;

    private DriveBase       driveBase;
    private Trajectory      trajectory;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    private static double   kRamseteB = 2, kRamseteZeta = .7;
 
    // Estimate feed forward gains as 12v / max velocity.
    //private static double   ksVolts = 12 / MAX_WHEEL_SPEED_MS, kvVoltSecondsPerMeter = 12 / MAX_WHEEL_SPEED_MS;
    private static double   ksVolts = .2, kvVoltSecondsPerMeter = 2;
    private static double   kaVoltSecondsSquaredPerMeter = .2;

    AutoDriveTrajectory(DriveBase driveBase, Trajectory trajectory)
    {
        super(trajectory,
                driveBase::getOdometerPose,
                new RamseteController(kRamseteB, kRamseteZeta),
                new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
                new DifferentialDriveKinematics(Util.inchesToMeters(TRACK_WIDTH)),
                driveBase::getWheelSpeeds,
                new PIDController(kP, kI, kD),
                new PIDController(kP, kI, kD),
                // RamseteCommand passes volts to the callback
                driveBase::setVoltage,
                driveBase);

        this.driveBase = driveBase;
        this.trajectory = trajectory;
    }
    
    @Override
    public void initialize()
    {
        Util.consoleLog();

        startTime = Util.timeStamp();

        super.initialize();

        // Synchronize driveBase odometer with trajectory initial pose.
        Pose2d pose = trajectory.getInitialPose();

        Util.consoleLog("initial poseX=%.2f  poseY=%.2f  poseHdg=%.2f", pose.getX(), pose.getY(), -pose.getRotation().getDegrees());
        
        pose = driveBase.getOdometerPose();

        Util.consoleLog("robot poseX=%.2f  poseY=%.2f  poseHdg=%.2f", pose.getX(), pose.getY(), -pose.getRotation().getDegrees());

        //driveBase.resetOdometer(pose, pose.getRotation().getDegrees());
    }

    @Override
    public void execute()
    {
        Util.consoleLog();

        super.execute();

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
        
        driveBase.stop();
                
        Pose2d pose = driveBase.getOdometerPose();

        Util.consoleLog("poseX=%.2f  poseY=%.2f  poseHdg=%.2f", pose.getX(), pose.getY(), -pose.getRotation().getDegrees());

		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

        Util.consoleLog("end ----------------------------------------------------------");
    }

    public static DifferentialDriveVoltageConstraint getVoltageConstraint()
    {
        return new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
                    new DifferentialDriveKinematics(Util.inchesToMeters(TRACK_WIDTH)),
                    10);    // 10v to leave some room to go over the constraint.
    }

    public static TrajectoryConfig getTrajectoryConfig(DifferentialDriveVoltageConstraint voltageConstraint)
    {
        return new TrajectoryConfig(MAX_WHEEL_SPEED_MS, MAX_WHEEL_ACCEL_MSS)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(new DifferentialDriveKinematics(Util.inchesToMeters(TRACK_WIDTH)))
                    // Apply the voltage constraint
                    .addConstraint(voltageConstraint);
    }
}
