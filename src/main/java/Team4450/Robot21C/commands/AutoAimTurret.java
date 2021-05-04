package Team4450.Robot21C.commands;

import Team4450.Lib.Util;
import Team4450.Robot21C.subsystems.LimeLight;
import Team4450.Robot21C.subsystems.Turret;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * A command that will aim the turret automatically based on LimeLight camera information.
 */
public class AutoAimTurret extends PIDCommand 
{
    private static double       kP = .10, kI = .01, kD = 0;
    private double              kToleranceDeg = 1.0;
    private Turret              turret;
    private LimeLight           limeLight;
    private boolean             targetVisible, targetLocked;
    /**
     * Turns turret to center the target in the LimeLight field of view.
     *
     * @param turret Turret subsystem to use.
     * @param limeLight LimeLight subsystem to use
     */
    public AutoAimTurret(Turret turret, LimeLight limeLight) 
    {
        super(
            new PIDController(kP, kI, kD),
            // Closed loop on target center X offset from center of field of view
            limeLight::offsetX,
            // Set target as zero (X offset from center of field of view)
            0,
            // Pipe output to turn turret
            output -> turret.rotate(output),
            // Require the turret
            turret);

        this.turret = turret;
        this.limeLight = limeLight;

        //getController().enableContinuousInput(-180, 180);

        getController().setTolerance(kToleranceDeg);
    }

    @Override
    public void initialize() 
    {
        Util.consoleLog();
        
        limeLight.selectPipeline(0);
        limeLight.setCameraMode(LimeLight.CameraMode.vision);
        limeLight.setLedMode(LimeLight.LedMode.on);

        updateDS();
    }

    @Override
    public void execute() 
    {
        Util.consoleLog();

        // Ask LimeLight subsystem to update its targeting information.
        targetVisible = limeLight.processImage();

        // If the target is visible in the field of view, execute the embedded PID command
        // to turn the turret to center the target image in field of view.
        if (targetVisible) super.execute();

        updateDS();
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b targetVisible=%b targetLocked=%b", interrupted, targetVisible,
                        targetLocked);

        // When we end, we are no longer targeting, even if we have found and locked the target.
        targetVisible = false;
        
        limeLight.setCameraMode(LimeLight.CameraMode.driver);
        limeLight.setLedMode(LimeLight.LedMode.off);

        updateDS();
    }

    @Override
    public boolean isFinished() 
    {
        // End if no target visible.
        if (!targetVisible) return true;

        // End when the controller is at the reference, that is target center at zero offset from
        // center of field of view.
        return targetLocked = getController().atSetpoint();
    }

	private void updateDS()
	{
		SmartDashboard.putBoolean("AutoTarget", targetVisible);
		SmartDashboard.putBoolean("TargetLocked", targetLocked);
	}
}