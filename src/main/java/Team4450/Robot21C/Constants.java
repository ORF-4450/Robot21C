
package Team4450.Robot21C;

import java.util.Properties;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static String		PROGRAM_NAME = "RAC21C-02.02.21-1";

	public static Robot			robot;

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false;
	    	
	public static DriverStation.Alliance	alliance;
	public static int                       location, matchNumber;
	public static String					eventName, gameMessage;
	    
	// Drive motor controller port assignments.
	public static final int		LF_TALON = 1, LR_TALON = 2, RF_TALON = 3, RR_TALON = 4;
	
	// Other motor controller port assignments
	public static final int		PICKUP_TALON = 6, BELT_TALON = 7, WINCH_FRONT_VICTOR = 8, WINCH_BACK_VICTOR = 9;
	public static final int		HOOK_VICTOR = 10, COLOR_WHEEL_VICTOR = 11;
	
	// Joystick port assignments.
	public static final int		LEFT_STICK = 0, RIGHT_STICK = 1, UTILITY_STICK = 2, LAUNCH_PAD = 3;

	// Pneumatic valve controller port assignments.
	public static final int		COMPRESSOR = 0;
	public static final int		HIGHLOW_VALVE = 0;			// 0-1 
	public static final int		PICKUP_VALVE = 2; 			// 2-3
	public static final int		CLIMBER_BRAKE_VALVE = 4;	// 4-5

	// Digital Input port assignments. Encoder takes 2 ports.
	public static final int		WINCH_SWITCH = 0, WINCH_ENCODER = 1, BALL_EYE = 3;

	// Dummy encoders use DIO port numbers above the actual ports on RoboRio.
	public static final int		DUMMY_LEFT_ENCODER = 10, DUMMY_RIGHT_ENCODER = 12;
	  
	// Analog Input port assignments.
	// Simulated Gyro needs an actual analog port and has to be 0 or 1.
	public static final int		SIM_GYRO = 0;
	public static final int		CLIMBER_GYRO = 1;
	public static final int		PRESSURE_SENSOR = 2;

	public static final DriverStation	ds = DriverStation.getInstance();

	public static final double	TALON_RAMP_RATE = 1.0;			// Takes 1 sec for full power to applied.
	public static final double  DRIVE_WHEEL_DIAMETER = 6.20;	// Inches.
	public static final double	COLORWHEEL_SPEED = .25;
	public static final int		COLORWHEEL_ROTATIONS = 3;
	public static final double	STEERING_ASSIST_GAIN = .05;
    public static final double	TRACK_WIDTH = 23;				// Inches. 
    public static final double  MAX_WHEEL_SPEED_MS = 1.5;
    public static final double  MAX_WHEEL_ACCEL_MSS = 1.5;
	
	// LCD display line number constants showing class where the line is set.
	public static final int		LCD_1 = 1;	// Robot, Auto Commands.
	public static final int		LCD_2 = 2;	// DriveCommand.
	public static final int		LCD_3 = 3;	// DriveCommand.
	public static final int		LCD_4 = 4;	// AutoDrive.
	public static final int		LCD_5 = 5;	// AutoDrive.
	public static final int		LCD_7 = 7;	// DriveCommand.
	public static final int		LCD_8 = 8;	// DriveCommand.
	public static final int		LCD_9 = 9;	// DriveCommand, pickup.

	// Default starting field position in meters for pose tracking.
	public static final double	INITIAL_X = 1.2;
	public static final double	INITIAL_Y = 0.5;
	public static final double	INITIAL_HEADING = 0;
}
