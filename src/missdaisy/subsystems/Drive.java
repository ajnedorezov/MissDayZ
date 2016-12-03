package missdaisy.subsystems;

import edu.wpi.first.wpilibj.Talon;
import missdaisy.Inputs;

public class Drive {
	private static Drive driveInstance;
	private Talon forwardDriveMotor; // points north 
	private Talon rightDriveMotor; //  points east
	private Talon backDriveMotor; //  points south
	private Talon leftDriveMotor; //  points west 
	
	public static Drive getInstance() {
		if (driveInstance == null) {
			driveInstance = new Drive();
		} 
		return driveInstance;
	}
	
	private Drive() {
		forwardDriveMotor = new Talon(Inputs.PWMs.FORWARD_DRIVE);
		rightDriveMotor = new Talon(Inputs.PWMs.RIGHT_DRIVE); 
		backDriveMotor = new Talon(Inputs.PWMs.BACK_DRIVE); 
		leftDriveMotor = new Talon(Inputs.PWMs.LEFT_DRIVE);
		rightDriveMotor.setInverted(true);
		backDriveMotor.setInverted(true);
	}
	
	public void setSpeed(double forward, double back) {
		set(forward, back, 0.0, 0.0);
	}

	public void setSpeedTurn(double speed, double turn) {
		double leftSpeed = speed + turn;
		double rightSpeed = speed - turn;
		set(leftSpeed, rightSpeed, 0.0, 0.0);
	}
	
	public void setStrafe(double left, double right) {
		set(0.0, 0.0, left, right);
	}
	
	public void turnInPlace(double turn) {
		set(-turn, turn, -turn, turn);
	}
	
	
	/**
	 * This function allows you to set the speed of each drive motor individually.
	 * All inputs should be a number between -1.0 and 1.0.
	 * 
	 * @param forward 
	 * @param back
	 * @param left
	 * @param right
	 */
	public void set(double forward, double back, double left, double right) {
		forwardDriveMotor.set(forward);
		backDriveMotor.set(back);
		leftDriveMotor.set(left);
		rightDriveMotor.set(right);
	}
	
	public void reset() {
		set(0.0, 0.0, 0.0, 0.0);
	}
}
