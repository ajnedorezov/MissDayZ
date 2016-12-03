package missdaisy;

import missdaisy.controllers.DriveTurnController;
import missdaisy.loops.SynchronousPID;
import missdaisy.subsystems.Drive;
import missdaisy.util.DaisyMath;
import missdaisy.util.XboxController;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OperatorInput {
	private Drive drive;
	private XboxController driveController;
	private AHRS mGyro;
	
	private double yawSetPoint;
	
	private SynchronousPID turnPID;
	
	public OperatorInput() {
		drive = Drive.getInstance();
		driveController = new XboxController(Inputs.XboxControllerPorts.DRIVE_CONTROLLER);
		
		yawSetPoint = 0.0;
		
		try {
            /* Communicate w/navX MXP via the MXP SPI Bus.                                     */
            /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
            mGyro = new AHRS(SPI.Port.kMXP); 
            
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
		turnPID = new SynchronousPID((1/180)*2.0, -0.1, 0.01);
		turnPID.setContinuous();
		turnPID.setInputRange(-180, 180);
		SmartDashboard.putNumber("TurnP", -0.05);
		SmartDashboard.putNumber("TurnI", 0.0);
		SmartDashboard.putNumber("TurnD", -.1);

		SmartDashboard.putNumber("GyroHeading", 0.0);
	}
	
	public void processInputs() {
		// drive values. Multiplied by -1 because by default the xbox controller's foward direction
		// gives a negative value.
		double northSouth = DaisyMath.applyDeadband(driveController.getLeftYAxis(), 0.1); // u component
		// strafe values
		double eastWest = -DaisyMath.applyDeadband(driveController.getRightXAxis(), 0.1); // v component

		/*
		if (driveController.getLeftTrigger()) {
			drive.turnInPlace(0.5);
			//drive.setStrafe(1.0, 1.0);
		} else if (driveController.getRightTrigger()) {
			drive.turnInPlace(-0.5);
			//drive.setStrafe(-1.0, -1.0);
		} else if (driveController.getLB()){
			drive.turnInPlace(0.5);
		} else if (driveController.getRB()){
			drive.turnInPlace(-0.5);
		} else {
			//drive.setSpeed(forwardBack, forwardBack);
			//drive.setStrafe(leftRight, leftRight);
			//drive.set(forwardBack, forwardBack, leftRight, leftRight);
			//drive.setSpeedTurn(forwardBack, leftRight);
		}
		*/
		
		double speed = Math.hypot(northSouth, eastWest);
		if (driveController.getLeftTrigger()){
			yawSetPoint = DaisyMath.boundAngleNeg180to180Degrees(yawSetPoint - 1);
			turnPID.setSetpoint(yawSetPoint);
		} else if (driveController.getRightTrigger()) {
			yawSetPoint = DaisyMath.boundAngleNeg180to180Degrees(yawSetPoint + 1);
			turnPID.setSetpoint(yawSetPoint);
		}
		
		double desiredAngle = 180.0/Math.PI*Math.atan2(eastWest, northSouth);
		double currentAngle = mGyro.getYaw();
		SmartDashboard.putNumber("speed", speed);
		SmartDashboard.putNumber("desiredAngle", desiredAngle);
		SmartDashboard.putNumber("GyroHeading", currentAngle);
		
		double angle = -DaisyMath.boundAngleNeg180to180Degrees(desiredAngle - currentAngle);
		SmartDashboard.putNumber("angle", angle);
		
		double frontBack = speed*Math.cos(angle*Math.PI/180.0);
		double leftRight = speed*Math.sin(angle*Math.PI/180.0);
		SmartDashboard.putNumber("frontBack", frontBack);
		SmartDashboard.putNumber("lefRight", leftRight);
		
		double newP = SmartDashboard.getNumber("TurnP", 0);
		double newI = SmartDashboard.getNumber("TurnI", 0);
		double newD = SmartDashboard.getNumber("TurnD", 0);
		turnPID.setPID(newP, newI, newD);
		double actualP = turnPID.getP();
		double actualI = turnPID.getI();
		double actualD = turnPID.getD();
		SmartDashboard.putNumber("ActualTurnPIDValues", turnPID.getP());
		
		double yawError = DaisyMath.boundAngleNeg180to180Degrees(yawSetPoint - currentAngle);
		double turn = turnPID.calculate(currentAngle);
		
		//double turn = 1.0/180.0*yawError/4.0;
		SmartDashboard.putNumber("yawSetPoint", yawSetPoint);
		SmartDashboard.putNumber("yawError", yawError);
		SmartDashboard.putNumber("turn", turn);
		
		drive.set(frontBack - turn, frontBack + turn, leftRight - turn, leftRight + turn);
		
	}
}
