import lejos.nxt.*;

/** Class that determines the identity of an unknown obstacle
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  1.0 November 2 2013
 */
public class ObjectDetection {

	private UltrasonicSensor us1;
	private UltrasonicSensor us2;
	private int samples = 20;
	
	private final int BLOCK_DISTANCE = 45;
	
	/** Constructor method to import the two ultrasonic sensors
	 * 
	 * @param us1 import the ultrasonic sensor located on the bottom of the robot
	 * @param us2 import the ultrasonic sensor located at the top of the robot
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
		
		int distanceUS2;
		boolean falseAlarm = false;
		
		distanceUS2 = getFilteredData(us2);
		
		if (distanceUS2 < BLOCK_DISTANCE) {
			for(int i = 0; i < samples; i++) {
				if(getFilteredData(us1) < BLOCK_DISTANCE + 10) {
					falseAlarm = true;
				}
				if(getFilteredData(us2) > BLOCK_DISTANCE) {				
					falseAlarm = true;
				}
			}
			if(!falseAlarm) {
				return true;					
			}
		}
		
		return false;
	}
	
	public boolean isObstacle() {
		
		int distance;
		
		distance = getFilteredData(us1);
		
		if(distance < BLOCK_DISTANCE) {
			Sound.buzz();
			return true;
		}
		
		return false;
	}
	
	/** Method to get filtered data from the ultrasonic sensors
	 * 
	 * @param us imports the ultrasonic sensor we want to find the filtered data for
	 * @return returns filtered data read by the ultrasonic sensor
	 */
	public int getFilteredData(UltrasonicSensor us){
		
		int distance;
		
		// ping the sensor
		us.ping();
		
		// wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		
		// after 50ms get the distance and return it
		distance = us.getDistance();		
		return distance;
	}
}
