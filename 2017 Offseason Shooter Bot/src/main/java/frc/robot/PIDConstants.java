/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

public class PIDConstants {
	/**
	 * Set to zero to skip waiting for confirmation.
	 * Set to nonzero to wait and report to DS if action fails.
	 */
    public final static int kTimeoutMs = 30;
    
    //DriveTrain Motion Magic Constants
    public final static int kDriveTrainSensorVel = 1000;
    public final static int kDriveTrainAccel = kDriveTrainSensorVel/2;
    public final static int kDriveTrainVelocity = kDriveTrainSensorVel/2;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 
	 * 	                                    	       kP   kI   kD   kF                          Iz   PeakOut */
	public final static Gains kGains_Drive = new Gains( 0.0, 0.0,  0.0, 0.0, 100,  0.50 );


	
	/** ---- Flat constants, you should not need to change these ---- */
	/* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Drive = 0;

}
