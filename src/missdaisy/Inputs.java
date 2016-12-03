package missdaisy;

/**
 * All the inputs to the roborio
 */
public class Inputs {
	public static class PWMs {
		public static final int FORWARD_DRIVE = 0; // points north 
		public static final int RIGHT_DRIVE = 1; //  points east
		public static final int BACK_DRIVE = 2; //  points south
		public static final int LEFT_DRIVE = 3;
	}
	
	public static class XboxControllerPorts {
		public static final int DRIVE_CONTROLLER = 0;
	}
	
	public static class Properties {
		public static final double PID_DRIVE_ANGLE_TOLERANCE = 5;
		public static final double PID_DRIVE_TURN_MIN_OUTPUT = 0;
	}
}
