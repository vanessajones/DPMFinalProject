
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.TouchSensor;

public class Navigation {
	final static int FAST = 200, SLOW = 140, ACCELERATION = 4000;
	final static double DEG_ERR = 3.0, CM_ERR = 1.0;
	
	private Odometer odometer;
	private UltrasonicSensor us;
	private NXTRegulatedMotor leftMotor, rightMotor;
	
	private final double RADIUS = 2.15;
	private final double WIDTH = 15.8;

	
	private boolean lookingForObject;
	private boolean isNavigating;
	
	private int leftWallBound;
	private int rightWallBound;
	private int topWallBound;
	private int bottomWallBound;
	private int[] forbiddenZoneX;
	private int[] forbiddenZoneY;
	
	private int role;
	
	public Navigation(Odometer odo, UltrasonicSensor us, int startingLocation, int role) {
		this.odometer = odo;
		this.us = us;
		
		this.leftMotor = Motor.A;
		this.rightMotor = Motor.B;

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
		
		this.setBoundaries(startingLocation, role);
	}
	
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	/*
	 * Float the two motors jointly
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/*
	 * TravelTo function which takes as arguments the x and y position in cm. Will travel to designated position, while
	 * constantly updating it's heading
	 */
	public void travelTo(double x, double y) {
		isNavigating = true;
		double minAng;
		while (Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) {	
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			this.turnTo(minAng, false);
			this.setSpeeds(SLOW, SLOW);
		}
		this.setSpeeds(0, 0);
		isNavigating = false;
	}

	/*
	 * TurnTo function which takes an angle and boolean as arguments The boolean controls whether or not to stop the
	 * motors when the turn is completed
	 */
	public void turnTo(double angle, boolean stop) {

		double error = angle - this.odometer.getAng();

		while (Math.abs(error) > DEG_ERR) {

			error = angle - this.odometer.getAng();

			if (error < -180.0) {
				this.setSpeeds(-SLOW, SLOW);
			} else if (error < 0.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else if (error > 180.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else {
				this.setSpeeds(-SLOW, SLOW);
			}
		}

		if (stop) {
			this.setSpeeds(0, 0);
		}
	}

	/* method which tells the robot to travel a certain distance, if boolean wait is false, the method
	   will return immediately */
	public int travelBy(double dist, boolean wait) {
		if(wait) {
			leftMotor.rotate(convertDistance(RADIUS, dist), true);
			rightMotor.rotate(convertDistance(RADIUS, dist), false);
		}
		else {
			leftMotor.rotate(convertDistance(RADIUS, dist), true);
			rightMotor.rotate(convertDistance(RADIUS, dist), true);
		}
		return 0;
	}
	
	/* method which tells the robot to robot a angle, if boolean wait is false, the method
	   will return immediately */
	public int rotateBy(double angle, boolean wait) {
		if(wait) {
			leftMotor.rotate(convertAngle(RADIUS, WIDTH, angle), true);
			rightMotor.rotate(-convertAngle(RADIUS, WIDTH, angle), false);
		}
		else {
			leftMotor.rotate(convertAngle(RADIUS, WIDTH, angle), true);
			rightMotor.rotate(-convertAngle(RADIUS, WIDTH, angle), true);
		}
		return 0;
	}
	
	public void ScanForObject() {

	}


	public void setBoundaries(int startingLocation, int role) {
		if(startingLocation == 1 | startingLocation == 3) {
			leftWallBound = -25;
			rightWallBound = 325;
			topWallBound = 325;
			bottomWallBound = -25;
		}
		else {
			leftWallBound = -325;
			rightWallBound = 25;
			topWallBound = 325;
			bottomWallBound = -25;
		}
		
		if(startingLocation == 1) {
			if(role == 1) {
				forbiddenZoneX = new int[] { /*blah*/ };
				forbiddenZoneY = new int[] { /*blah*/ };
			}
			else {
				forbiddenZoneX = new int[] { /*blah*/};
				forbiddenZoneY = new int[] { /*blah*/ };				
			}
		}
		else if(startingLocation == 2) {
			if(role == 1) {
				forbiddenZoneX = new int[] { /*blah*/ };
				forbiddenZoneY = new int[] { /*blah*/ };				
			}
			else {
				forbiddenZoneX = new int[] { /*blah*/};
				forbiddenZoneY = new int[] { /*blah*/ };				
			}
		}
		else if(startingLocation == 3) {
			if(role == 1) {
				forbiddenZoneX = new int[] { /*blah*/ };
				forbiddenZoneY = new int[] { /*blah*/ };					
			}
			else {
				forbiddenZoneX = new int[] { /*blah*/ };
				forbiddenZoneY = new int[] { /*blah*/ };				
			}
		}
		else{
			if(role == 1) {
				forbiddenZoneX = new int[] { /*blah*/ };
				forbiddenZoneY = new int[] { /*blah*/ };					
			}
			else {
				forbiddenZoneX = new int[] { /*blah*/ };
				forbiddenZoneY = new int[] { /*blah*/ };					
			}
		}
	}
	
	
/*	
	public boolean isBoundary() {
	
		double distance;
			
		us.ping();
		// wait for ping to finish
		try { Thread.sleep(25); } catch (InterruptedException e) {}
		distance = getFilteredData();
			
		double angle = odometer.getAng();
		double x = odometer.getX() + distance*Math.cos(Math.toRadians(angle));
		double y = odometer.getY() + distance*Math.sin(Math.toRadians(angle));
		
		// wall boundaries
		if(x < leftWallBound || x > rightWallBound || y < bottomWallBound || y > topWallBound) {
			return true;
		}
				
		return false;
	}
*/
	
	/* called only when robot identifies a styrofoam block */
	public void grabBlock() {

	}
	
	public boolean foundObject() {
		return this.foundObject();
	}
	
	public boolean lookingForObject() {
		return this.lookingForObject;
	}
	
	public boolean isNavigating() {
		return this.isNavigating;
	}
	
	private int getFilteredData() {
		int distance;
		
		// do a ping
		us.ping();
		
		// wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		
		// there will be a delay here
		distance = us.getDistance();

		if (distance > 80)
			distance = 80;
		
		return distance;

	}
	
	// Useful conversion methods taken from Lab2 SquareDriver.java file
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private static int convertDistance(double radius, double vector_magnitude) {
		return (int) (( vector_magnitude * 180.0) / (Math.PI * radius));
	}
			
	
}
