import lejos.nxt.*;

/** Class that determines the identity of an unknown obstacle
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  1.0 November 2 2013
 */
public class ObjectDetection extends Exception {
	
	class ObjectNotFoundException extends Exception{};

	private UltrasonicSensor us1;
	private UltrasonicSensor us2;
	
	private final int SAMPLES = 5;
	private final int FALSE_NEGATIVE = 20;
	private final int BLOCK_DISTANCE = 45;
	
	private int filterControl = 0;
	
	private Odometer odometer;
	
	/** Constructor method to import the two ultrasonic sensors
	 * 
	 * @param us1 import the ultrasonic sensor located on the TOP of the robot
	 * @param us2 import the ultrasonic sensor located at the BOTTOM of the robot
	 * @param odo import the odometer that is also used by the navigation class
	 */
	public ObjectDetection(UltrasonicSensor us1, UltrasonicSensor us2, Odometer odo){
		
		this.us1 = us1;
		this.us2 = us2;
		
		this.odometer = odo;
		
	}
	
	/** Finds location of a styrofoam block and gives the coordinates for the block
	 * 
	 * @return the x and y coordinates of a styrofoam block
	 * @exception ObjectNotFoundException if no styrofoam blocks are found after one run-through
	 */
	public double[] findObject() throws ObjectNotFoundException{
		double[] coords = new double[3];
		
		// check if there is a blue block ahead
		boolean styrofoam = isBlue();
		
		if (styrofoam) {
			// import x, y, and theta from odometer
			coords = odometer.getPosition();
			
			// find x and y position using distance and angle
			int distance = getFilteredData(us2);
			coords[0] += (double) distance * Math.cos(Math.toRadians(coords[2]));
			coords[1] += (double) distance * Math.sin(Math.toRadians(coords[2]));
		}
		
		// otherwise throw ObjectNotFoundException
		else {
			throw new ObjectNotFoundException();
		}
		
		// return the updated coordinates to navigate to
		return coords;
	}
	
	/** Determines the identity of an unknown block
	 * 
	 * @return the identity of the obstacle. Returns true is the block is a blue Styrofoam block and false otherwise.
	 */
	public boolean isBlue(){
		
		int distanceUS2;
		boolean falseAlarm = false;
		
		distanceUS2 = getFilteredData(us2);
		
		// if the bottom sensor detects something sample the top sensor a few times to see if it's a false alarm
		if (distanceUS2 < BLOCK_DISTANCE) {
			for(int i = 0; i < SAMPLES; i++) {
				if(getFilteredData(us1) < BLOCK_DISTANCE) {
					falseAlarm = true;
				}
			}
			// if not and the bottom sensor can still detect the block then it must be real
			if(!falseAlarm && (getFilteredData(us2) < BLOCK_DISTANCE)) {
				return true;					
			}
		}
		
		return false;
	}
	
	/** Determines if there is an obstacle ahead
	 * 
	 * @return true if there is an obstacle, false if otherwise
	 */
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
	
	public int getFilteredData(){
		
		int distance;
		
		// ping the sensor
		us2.ping();
		
		// wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		
		// after 50ms get the distance and return it
		distance = us2.getDistance();		
		return distance;
	}
	
	public int falseNeg() {
		int distance;
		int safeValue = 30;
		
		us2.ping();
		
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		
		distance = us2.getDistance();	
		
		if(distance < 50) {
			filterControl = 0;
			return distance;
		}
		else if(distance >= 50 && filterControl < FALSE_NEGATIVE) {
			return safeValue;
		}
		else {
			return distance;
		}
		
	}
	
	
}
