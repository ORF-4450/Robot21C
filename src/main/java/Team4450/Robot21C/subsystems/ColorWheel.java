package Team4450.Robot21C.subsystems;

import static Team4450.Robot21C.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatchResult;

import Team4450.Lib.RevColorSensor;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Color Wheel subsystem.
 */
public class ColorWheel extends SubsystemBase
{
	private boolean					wheelRunning, rotatingToTarget;
	
  	private RevColorSensor			colorSensor = RevColorSensor.getInstance();	  
  	
  	private static WPI_VictorSPX	colorWheelVictor;

	public ColorWheel()
	{
		Util.consoleLog();

		colorWheelVictor = new WPI_VictorSPX(COLOR_WHEEL_VICTOR);

		Util.consoleLog("ColorWheel created!");
	}
	
	// This method will be called once per scheduler run
	@Override
	public void periodic() 
	{
	}

	private void updateDS()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Wheel", wheelRunning);
	}

	/**
	 * Stop color wheel.
	 */
	public void stopWheel()
	{
		Util.consoleLog();
		
		//if (countingTurns) stopCountingTurns();
		
		//if (rotatingToTarget) stopRotateToTarget();

		colorWheelVictor.stopMotor();
		
		wheelRunning = false;
		
		updateDS();
	}
	
	/**
	 * Toggle color wheel turning at default power.
	 */
	public void toggleWheel()
	{
		Util.consoleLog();
		
		if (isRunning())
			stopWheel();
		else
			startWheel(COLORWHEEL_SPEED);
	}
	
	/**
	 * Start color wheel turning.
	 * @param power Power level 0.0 to 1.0.
	 */
	public void startWheel(double power)
	{
		Util.consoleLog();
		
		colorWheelVictor.set(power);
		
		wheelRunning = true;
		
		updateDS();
	}
	
	/**
	 * Returns running state of wheel.
	 * @return True is running.
	 */
	public boolean isRunning()
	{
		return wheelRunning;
	}
	
	/**
	 * Sets target color to current color under sensor.
	 */
	public void setTargetToCurrentColor()
	{
		Color targetColor = colorSensor.getColor();
		
		Util.consoleLog(targetColor.toString());
		
		colorSensor.resetColorMatcher();
		
		if (targetColor != null) colorSensor.addColorMatch(targetColor);
	}
	
	/**
	 * Convert single letter color code to full color word.
	 * @param gameColor
	 * @return
	 */
	public static String convertGameColor(String gameColor)
	{
		String color = "";
		
		if (gameColor == null) return color;
		
		if (gameColor.length() == 0) return color;
		
		switch (gameColor.charAt(0))
		{
			case 'B' :
				color = "BLUE";
				break;
				
			case 'G' :
				color = "GREEN";
				break;
				
			case 'R' :
				color = "RED";
				break;
				
			case 'Y' :
				color = "YELLOW";
				break;
				
			default :
				color = "";
				break;
	  }

		return color;
	}
	
	/**
	 * Reads game data and if color code present, loads the correct Color
	 * object (target color) into the Color Sensor matching function.
	*/
	public void setGameTargetColor()
	{
		Color	targetColor = null;
		String 	gameData = DriverStation.getInstance().getGameSpecificMessage(), color = "";
		
		gameData = "R";
		
		Util.consoleLog(gameData);
		
		color = convertGameColor(gameData);
		
		colorSensor.resetColorMatcher();

		if(gameData != null && gameData.length() > 0)
		{
			switch (gameData.charAt(0))
			{
				case 'B' :
					//Blue code. Match color is Red.
					targetColor = RevColorSensor.getMatchColor(0.561, 0.232, 0.114);	// Red
					//targetColor = RevColorSensor.getMatchColor(0.143, 0.427, 0.429);	// Blue
					break;
					
				case 'G' :
					//Green code. Match color is Yellow.
					targetColor = RevColorSensor.getMatchColor(0.361, 0.524, 0.113);	// Yellow
					//targetColor = RevColorSensor.getMatchColor(0.197, 0.561, 0.240);	// Green
					break;
					
				case 'R' :
					//Red code. Match color is Blue.
					targetColor = RevColorSensor.getMatchColor(0.143, 0.427, 0.429);	// Blue
					//targetColor = RevColorSensor.getMatchColor(0.561, 0.232, 0.114);	// Red
					break;
					
				case 'Y' :
					//Yellow code. Match color is Green.			
					targetColor = RevColorSensor.getMatchColor(0.197, 0.561, 0.240);	// Green
					//targetColor = RevColorSensor.getMatchColor(0.361, 0.524, 0.113);	// Yellow
					break;
					
				default :
					//This is corrupt data.
					break;
		  }
		} else {
			// no data received yet so no match.
		}		

		SmartDashboard.putString("GameColor", color);

		if (targetColor != null) colorSensor.addColorMatch(targetColor);
	}
	
	/**
	 * Match current color read from sensor to the target color in the color matcher
	 * and returns true if current color matches target color with confidence >= 85%.
	 * @return True if sensor color matches target color.
	 */
	public boolean colorMatch()
	{
		Color color = colorSensor.getColor();

		ColorMatchResult matchResult = colorSensor.matchClosestColor(color);

		//LCD.printLine(6, "color match result r=%f g=%f b=%f  conf=%f", matchResult.color.red, matchResult.color.green,
		//		matchResult.color.blue, matchResult.confidence);

		if (matchResult.confidence >= .85)
			return true;
		else
			return false;
	}

}
