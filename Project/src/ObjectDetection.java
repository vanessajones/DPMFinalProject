import lejos.nxt.*;

/** Class that determines the identity of an unknown obstacle
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  1.0 November 2 2013
 */
public class ObjectDetection {

	private UltrasonicSensor us1;
	private UltrasonicSensor us2;
	
	/** Constructor method to import the two ultrasonic sensors
	 * 
	 * @param us1 import the ultrasonic sensor located on the top of the robot
	 * @param us2 import the ultrasonic sensor located at the bottom of the robot
	 */
	public ObjectDetection(UltrasonicSensor us1, UltrasonicSensor us2){
		
		this.us1 = us1;
		this.us2 = us2;
		
	}
	
	/** Determines the identity of an unknown block
	 * 
	 * @return the identity of the obstacle. Returns true is the block is a blue Styrofoam block and false otherwise.
	 */
	public boolean isBlue(){
	
	}
	
	/** Method to get filtered data from the ultrasonic sensors
	 * 
	 * @return returns filtered data read by the ultrasonic sensor
	 */
	public int getFilteredData(){
		
	}
}
