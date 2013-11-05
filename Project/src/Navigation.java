
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.TouchSensor;
import lejos.nxt.comm.Bluetooth;

/** Class that houses the commands related to navigation.
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  1.0 November 2 2013
 */

public class Navigation {
	final static int FAST = 200, SLOW = 140, ACCELERATION = 4000;
	final static double DEG_ERR = 3.0, CM_ERR = 1.0;
	
	private Odometer odometer;
	private NXTRegulatedMotor leftMotor, rightMotor;
	private Bluetooth bt;
	private LightLocalizer liLocalizer;
	private ObjectDetection objDetection;
	
	private final double RADIUS = 2.18;
	private final double WIDTH = 15.8;
	private final double TILELENGTH = 30;
	
	private boolean hasBlock = false;
	private boolean foundBlock = false;
	private boolean justWentStraight = false;
	
	private final int leftWallBound = -25;
	private final int rightWallBound = 325;
	private final int topWallBound = 325;
	private final int bottomWallBound = -25;
	private int[] restrictedAreaX;
	private int[] restrictedAreaY;
	
	private int role;
	private int startingLocation;
	
	/** Class constructor
	 * 
	 * @param odo imports Odometer 
	 * @param bt imports Bluetooth
	 * @param ls imports LightLocalizer
	 * @param od imports ObjectDetection
	 */
	
	public Navigation(Odometer odo, Bluetooth bt, LightLocalizer ls, ObjectDetection od) {
		this.odometer = odo;
		this.bt = bt;
		this.liLocalizer = ls;
		this.objDetection = od;
		
		this.leftMotor = Motor.A;
		this.rightMotor = Motor.B;

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
/*		
		role = bt.getRole();
		startingLocation = bt.getStartingLocation();
		restrictedAreaX = bt.getrestrictedAreaX;
		restrictedAreaY = bt.getrestrictedAreaY;
*/
	}
	
	
	/** Sets the travel speed of the robot
	 * 
	 * @param lSpd left motor speed
	 * @param rSpd right motor speed
	 */
	
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
	
	/** Sets the travel speed of the robot
	 * 
	 * @param lSpd left motor speed
	 * @param rSpd right motor speed
	 */

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

	/** Float the two motors jointly
	 * 
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}
	
	/** Method that tells the robot to travel in search for blue blocks
	 * 
	 */
	public void goFindBlock() {
		while(!foundBlock) {
			if(justWentStraight) {
				turnRight();
				/*if ObjectDetection found a block as robot was rotating, 
				 * 	  foundBlock = true;
				 *    call goGrabBlock
				 *    break
				 */
				pickSafeRoute();
				traverseATile();
				justWentStraight = false;
			}
			else {
				turnLeft();
				/*if ObjectDetection found a block as robot was rotating, 
				 * 	  foundBlock = true;
				 *    call goGrabBlock
				 *    break
				 */
				pickSafeRoute();
				traverseATile();
				justWentStraight = true;
			}
		}
	}
	
	public void pickSafeRoute() {
		boolean notSafe = true;
		int errorMargin = 5;
		int order;
		int destX, destY;
		while(notSafe) {
			
			if(getRobotDirection().equals("east")) {
				destX = (int) ((((odometer.getX() + errorMargin) % TILELENGTH) + 1)*TILELENGTH);
				destY = (int) (((odometer.getY() + errorMargin) % TILELENGTH)*TILELENGTH);
			}
			else if(getRobotDirection().equals("north")) {
				destX = (int) (((odometer.getX() + errorMargin) % TILELENGTH)*TILELENGTH);
				destY = (int) ((((odometer.getY() + errorMargin) % TILELENGTH) + 1)*TILELENGTH);
			}	
			else if(getRobotDirection().equals("west")) {
				destX = (int) ((((odometer.getX() + errorMargin) % TILELENGTH) - 1)*TILELENGTH);
				destY = (int) (((odometer.getY() + errorMargin) % TILELENGTH)*TILELENGTH);
			}			
			
			else if(getRobotDirection().equals("south")) {
				destX = (int) (((odometer.getX() + errorMargin) % TILELENGTH)*TILELENGTH);
				destY = (int) ((((odometer.getY() + errorMargin) % TILELENGTH) - 1)*TILELENGTH);
			}		
			
		/* if(ObjectDetection doesn't see an obstacle in the way) {
		 *    order = isBoundary(destX,destY);
		 *    if(order == 0) {
		 *       notSafe = false;
		 *    }
		 *    else if(order == 1) {
		 *       if(justWentStraight) {
		 *          turnLeft();
		 *       }
		 *       else {
		 *          turnRight();
		 *       }
		 *    else {
		 *       rotateBy(180,false);
		 *    }
		 * else {
		 *    if(justWentStraight) {
		 *       turnLeft();
		 *    }
		 *    else {
		 *       turnRight();
		 *    }
		 * }
		*/

		}
	}
	
	public String getRobotDirection () {
		if(odometer.getAng() < 45 && odometer.getAng() > 315) {
			return "east";
		}
		else if(odometer.getAng() < 135 && odometer.getAng() > 45) {
			return "north";
		}
		else if(odometer.getAng() < 225 && odometer.getAng() > 135) {
			return "west";
		}
		else {
			return "south";
		}
	}
	
	public void traverseATile() {
	// cover most of the tile quickly, then slow down for light localizer
		setSpeeds(FAST,FAST);
		travelBy((TILELENGTH - 5),false);
		setSpeeds(SLOW,SLOW);
		/*
		ping lightsensors until one of the sensors finds a blackline, cut the motors and call 
		localizer method.
		After localizing, ping a short distance in front for a blue block,
		if found,
			foundBlock = true
			call goGrabBlock
		 */
	}
	
	public void goGrabBlock() {
		
	}
	
	public void howToApproachDropZone() {
		
	}
	
	public void bringToDropZone() {
		
	}
	
	/**
	 * TravelTo function which takes as arguments the x and y position in cm. Will travel to designated position, while
	 * constantly updating it's heading
	 * @param x x-coordinate of travel destination
	 * @param y y-coordinate fo travel destination
	 */
	public void travelTo(double x, double y) {
		double minAng;
		while (Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) {	
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			this.turnTo(minAng, false);
			this.setSpeeds(SLOW, SLOW);
		}
		this.setSpeeds(0, 0);
	}

	/**
	 * Tells robot to rotate to a specific angle.
	 * @param angle the angle the robot will rotate to.
	 * @param stop controls whether or not to stop the motors when the turn is completed.
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

	/** Method which tells the robot to travel a certain distance, if boolean wait is false, the method will return immediately.
	 * @param dist the distance the robot will travel by.
	 * @param wait if false, method will return immediately.
	 */
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
	
	/** Method which tells the robot to rotate a certain angle, if boolean wait is false, the method will return immediately.
	 *  @param angle the angle the robot will rotate by.
	 *  @param wait if false, method will return immediately.
	 *  */
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
	
	public void turnRight() {
		rotateBy(90,false);
	}
	
	public void turnLeft() {
		rotateBy(-90,false);
	}
	
	/**
	 * Determines if travel path will collide with a wall or forbidden zone.
	 * @param destX x-coordinate of travel path.
	 * @param destY y-coordinate of travel path.
	 * @return true if travel path is bad, false if its good.
	 */
	public int isBoundary(int destX, int destY) {
	
		// wall boundaries
		if(destX < leftWallBound || destX > rightWallBound || destY < bottomWallBound || destY > topWallBound) {
			return 1;
		}
		
		for(int i = 0; i < restrictedAreaX.length; i++) {
			if(restrictedAreaX[i] == destX && restrictedAreaY[i] == destY) {
				return 2;
			}
		}
				
		return 0;
	}
	
	/** A set method for if the robot's found a blue block. Called by the ObjectDetection class
	 * @return true if it's found a blue block, false otherwise.
	 */
	public void setfoundBlock(boolean found) {
		this.foundBlock = found;
	}
	
	
	/** Interprets how much the wheel motors should rotate to turn the robot a certain angle based on the robot's specifications.
	 * 
	 * @param radius the radius of the wheels.
	 * @param width the width between the two wheels.
	 * @param angle the desired angle.
	 * @return the angle to rotate the motors by
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/** Interprets how much the wheel motors should rotate to move the robot a certain distance based on the robot's specifications.
	 * 
	 * @param radius the radius of the wheels.
	 * @param vector_magnitude the desired travel distance.
	 * @return the angle to rotate the motors by.
	 */
	private static int convertDistance(double radius, double vector_magnitude) {
		return (int) (( vector_magnitude * 180.0) / (Math.PI * radius));
	}
			
	
}
