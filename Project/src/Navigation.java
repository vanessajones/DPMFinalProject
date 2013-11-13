/*Note: require light localizer method that will localize by crossing a gridline
 *      require object detection method that will return if there's an obstacle directly in front of robot; not sufficient
 *      to just ping ahead since robot is too wide to cover. Require method that will ping for blue block and then go to it
 *      if one is found.
 */


import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.Bluetooth;

import java.lang.Math;


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
//	private Bluetooth bt;
	private LightLocalizer liLocalizer;
	private ObjectDetection objDetection;
	private HandleBlock handle;
	
	private final double RADIUS = 2.2;
	private final double WIDTH = 20;
	private final double TILELENGTH = 30;
	private final int ERRORMARGIN = 5;
	
	private boolean hasBlock = false;
	private boolean foundBlock = false;
	private boolean justWentStraight = true;
	
	private final int leftWallBound;
	private final int rightWallBound;
	private final int topWallBound;
	private final int bottomWallBound;
	private int[] restrictedAreaX;
	private int[] restrictedAreaY;
	private int[] dropzoneX = {30,60,30,60};
	private int[] dropzoneY = {120,120,150,150};
	
	private int closestDropZonePtX;
	private int closestDropZonePtY;
	private int numOfHorizontalMoves;
	private int numOfVerticalMoves;
	private int destX;
	private int destY;
	
	private int role;
	private int startingLocation;
		
	/** Class constructor
	 * 
	 * @param odo imports Odometer 
	 * @param bt imports Bluetooth
	 * @param ls imports LightLocalizer
	 * @param od imports ObjectDetection
	 */
	
	public Navigation(Odometer odo, ObjectDetection od, LightLocalizer ls, HandleBlock hb) {
		this.odometer = odo;
//		this.bt = bt;
		this.liLocalizer = ls;
		this.objDetection = od;
		this.handle = hb;
		
		this.leftMotor = Motor.A;
		this.rightMotor = Motor.B;

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
		
		// test code for milestone demo
/*
		int[] inputX = {120,150,120,150};
		this.dropzoneX = new int[inputX.length];

		int [] inputY = {120,120,150,150};
		this.dropzoneY = new int[inputY.length];
		
		
		for(int i = 0; i < dropzoneX.length; i++) {
			dropzoneX[i] = inputX[i];
			dropzoneY[i] = inputY[i];
		}
		*/
		
		leftWallBound = -25;
		rightWallBound = 85;
		topWallBound = 200;
		bottomWallBound = -25;
		
		//setSpeeds(SLOW,SLOW);
/*		
		role = bt.getRole();
		startingLocation = bt.getStartingLocation();
		restrictedAreaX = bt.getrestrictedAreaX;
		restrictedAreaY = bt.getrestrictedAreaY;
		dropzoneX = bt.getdropzone;
		dropzoneY = bt.getdropzone;
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
		double[] coords = new double[3];
		while(!foundBlock) {
			if(justWentStraight) {
				turnRight(false);
				
				while(leftMotor.isMoving() || rightMotor.isMoving()) {
					try {
						coords = objDetection.findObject();
						Sound.beep();
						leftMotor.stop();
						rightMotor.stop();
						foundBlock = true;
						goGrabBlock(coords);
					} catch (Exception e) {
						// no blue block found, keep going
					}
				}

				pickSafeRoute();
				traverseATile();
				justWentStraight = false;
				
			}
			else {
				turnLeft(false);
				
				while(leftMotor.isMoving() || rightMotor.isMoving()) {
					try {
						coords = objDetection.findObject();
						Sound.beep();
						leftMotor.stop();
						rightMotor.stop();
						foundBlock = true;
						goGrabBlock(coords);
					} catch (Exception e) {
						// no blue block found, keep going
					}
				}
				
				pickSafeRoute();
				traverseATile();
				justWentStraight = true;
			}
		}
		
		/* while(!foundBlock) {
                        if(justWentStraight) {
                                turnRight(false);
                                
                                foundBlock = scanForBlue();
                                if(foundBlock) {
                                        goGrabBlock(odometer.getX(), odometer.getY(), odometer.getAng(), objDetection.getFilteredData());
                                        break;
                                }

                                pickSafeRoute();
                                traverseATile();
                                justWentStraight = false;
                                
                        }
                        else {
                                turnLeft(false);
                                foundBlock = scanForBlue();
                                if(foundBlock) {
                                        goGrabBlock(odometer.getX(), odometer.getY(), odometer.getAng(), objDetection.getFilteredData());
                                        break;
                                }
                                pickSafeRoute();
                                traverseATile();
                                justWentStraight = true;
                        }
                }
		 * 
		 * 
		 */
	}
	
	public boolean scanAheadForObstacle() {
		double originalAngle = odometer.getAng();
		
		rotateBy(15,false);
		while(leftMotor.isMoving() || rightMotor.isMoving()) {
			if(objDetection.isObstacle()) {
				turnTo(originalAngle, false);
				return false;
			}
		}
		
		rotateBy(-30,false);
		while(leftMotor.isMoving() || rightMotor.isMoving()) {
			if(objDetection.isObstacle()) {
				turnTo(originalAngle, false);
				return false;
			}
		}
		
		turnTo(originalAngle,false);
		return true;
	}
	
	/**
	 * Looks for a line intersection that isn't blocked or out of bounds.
	 */

	public void pickSafeRoute() {
		boolean notSafe = true;
		int order;
		
		while(notSafe) {
			getNextLineIntersection();
		    order = isBoundary(destX,destY);
		    
		    // if its not a boundary or wall
		    if(order == 0) {
		    	
		    	// check if obstacle in way
			    if(scanAheadForObstacle()) {
			    	notSafe = false;
			    }
			    
			    else { 
			        if(justWentStraight) {
				        turnLeft(true);
				    }
				    else {
				        turnRight(true);
				    }			    	
			    }
		    }
		    
		    // if its a wall
		    else if(order == 1) {
		        rotateBy(180,true);
		        justWentStraight = false;

		    }
		    
		    //if its a boundary
		    else {
		        if(justWentStraight) {
		            turnLeft(true);
		        }
		        else {
		            turnRight(true);
		        }
		    }
		    
		}
	}	
	
	/**
	 * Stores the coordinates of the nearest line intersection in the robot's heading.
	 */
	public void getNextLineIntersection() {
		String heading = getRobotDirection();
		
		if(heading.equals("east")) {
			destX = (int) (((int)((odometer.getX() + ERRORMARGIN) / TILELENGTH) + 1)*TILELENGTH);
			destY = (int) ((int)((odometer.getY() + ERRORMARGIN) / TILELENGTH)*TILELENGTH);
		}
		else if(heading.equals("north")) {
			destX = (int) ((int)((odometer.getX() + ERRORMARGIN) / TILELENGTH)*TILELENGTH);
			destY = (int) (((int)((odometer.getY() + ERRORMARGIN) / TILELENGTH) + 1)*TILELENGTH);
		}	
		else if(heading.equals("west")) {
			destX = (int) (((int)((odometer.getX() + ERRORMARGIN) / TILELENGTH) - 1)*TILELENGTH);
			destY = (int) ((int)((odometer.getY() + ERRORMARGIN) / TILELENGTH)*TILELENGTH);
		}			
		
		else if(heading.equals("south")) {
			destX = (int) ((int)((odometer.getX() + ERRORMARGIN) / TILELENGTH)*TILELENGTH);
			destY = (int) (((int)((odometer.getY() + ERRORMARGIN) / TILELENGTH) - 1)*TILELENGTH);
		}	
	}
	
	/**
	 * Returns the general heading of the robot.
	 * @return north, south, east or west in string form.
	 */
	public String getRobotDirection () {
		double angle =  odometer.getAng();
		
		if(angle < 45 || angle > 315) {
			return "east";
		}
		else if(angle < 135 && angle > 45) {
			return "north";
		}
		else if(angle < 225 && angle > 135) {
			return "west";
		}
		else {
			return "south";
		}
	}
	
	/**
	 * Displaces the robot one tile. Relies on black line detection.
	 */
	public void traverseATile() {
	// cover most of the tile quickly, then slow down for light localizer
		setSpeeds(FAST,FAST);
		travelBy((TILELENGTH - 5),true);
		setSpeeds(SLOW,SLOW);
		travelBy(5,true);
		//setSpeeds(SLOW,SLOW);
		//liLocalizer.doLocalization();
		// replace following line with lightlocalizer code
		
		
		/*
		ping lightsensors until one of the sensors finds a blackline, cut the motors and call 
		localizer method.
		After localizing, ping a short distance in front for a blue block,
		if found,
			foundBlock = true
			call goGrabBlock
		 */
	}
	
	/**
	 * Moves the robot to the detected blue block, robot will travel to inputs after it's grabbed the block
	 */
	//INCOMPLETE
	public void goGrabBlock(double[] coords) {
		double[] currentPos = new double[] {odometer.getX(),odometer.getY(),odometer.getAng()};
		LCD.drawInt((int)coords[0]*100, 0, 0);
		LCD.drawInt((int)coords[1]*100, 0, 1);
		while (objDetection.getFilteredData()!=5){
			travelTo(coords[0],coords[1]);
		}
		handle.capture();
		handle.lift();
		travelTo(currentPos[0],currentPos[1]);
		turnTo(currentPos[2],true);
		/*int angleOne = (int)angle;
		int angleTwo;
		int angleTurnTo;
		
		Sound.buzz();
		Sound.buzz();
		
		setSpeeds(SLOW,SLOW);
		if(justWentStraight) {
			turnRight(false);
		}
		else {
			turnLeft(false);
		}
		while(leftMotor.isMoving() || rightMotor.isMoving() ) {
			if(objDetection.falseNeg() >= 50) {
				setSpeeds(0,0);
				Sound.buzz();
				Sound.buzz();
			}
		}
		
		angleTwo = (int)odometer.getAng();
		
		angleTurnTo = (angleOne + angleTwo)/2;
		
		setSpeeds(SLOW,SLOW);
		turnTo(angleTurnTo,false);
		travelBy(usDist,true);
		// grab block
		travelTo(x,y);*/
	}
	
	/**
	 * Gives the shortest path to the drop zone. Doesn't take obstacles and restricted areas into account. 
	 */
	public void getShortestPathToDropZone() {
		int PLACEHOLDER = 9001;
		int pathLengthX;
		int pathLengthY;
		int pathLength;
		int bestPath = PLACEHOLDER;
		
		for(int i = 0; i < dropzoneX.length; i++) {
			pathLengthX = (int)((dropzoneX[i] - odometer.getX() + ERRORMARGIN) / TILELENGTH);
			pathLengthY = (int)((dropzoneY[i] - odometer.getY() + ERRORMARGIN) / TILELENGTH);
			pathLength = Math.abs(pathLengthX) + Math.abs(pathLengthY);
			
			if(pathLength < bestPath) {
				closestDropZonePtX = dropzoneX[i];
				closestDropZonePtY = dropzoneY[i];
				numOfHorizontalMoves = pathLengthX;
				numOfVerticalMoves = pathLengthY;
				bestPath = pathLength;
			}
		}
	}
	
	/**
	 * Directs the robot along a path to the drop zone based on the calculations in the method getShortestPathToDropZone().
	 */
	// INCOMPLETE, requires behavior for bad path.
	public void bringToDropZone() {
		int order;
		Boolean isSafe;
		getShortestPathToDropZone();
		
		while (numOfVerticalMoves != 0 || numOfHorizontalMoves != 0) {
			isSafe = true;
			
			if(Math.abs(numOfVerticalMoves) >= Math.abs(numOfHorizontalMoves)) {
				if(numOfVerticalMoves > 0) {
					turnTo(90,false);
				}
				else {
					turnTo(270,false);
				}
			}
			else {
				if(numOfHorizontalMoves > 0) {
					turnTo(0,false);
				}
				else {
					turnTo(180,false);
				}
			}
			
			// call obstacle detection, if object in way, change safe to false
			getNextLineIntersection();
			order = isBoundary(destX,destY);
			if(order != 0) {
				isSafe = false;
			}
			if(isSafe) {
				traverseATile();
				updatePath();
			}
			// robot moves past obstacle and updates shortest path
			else {
				circumventObstacle();
				getShortestPathToDropZone();
				
			}
		}
		
		// bad code, replace later
		if(closestDropZonePtX == dropzoneX[0]) {
			turnTo(45, false);
		}
		else if(closestDropZonePtX == dropzoneX[1]) {
			turnTo(135, false);
		}
		else if(closestDropZonePtX == dropzoneX[2]) {
			turnTo(315, false);
		}
		else {
			turnTo(225, false);
		}
		
		travelBy(14, true);
		travelTo(closestDropZonePtX, closestDropZonePtY);
		rotateBy(45, true);
	}
	
	/**
	 * Updates the number of grid displacements required to reach drop zone.
	 */
	public void updatePath() {
		if(getRobotDirection().equals("east")) {
			numOfHorizontalMoves--;
		}
		else if(getRobotDirection().equals("north")) {
			numOfVerticalMoves--;
		}	
		else if(getRobotDirection().equals("west")) {
			numOfHorizontalMoves++;
		}			
		else if(getRobotDirection().equals("south")) {
			numOfVerticalMoves++;
		}			
	}
	
	public void circumventObstacle() {
		String heading = getRobotDirection();
		int angle;
		int originalHeading;
		boolean safe = false;
		
		if(heading.equals("north") || heading.equals("south")) {
			if(numOfHorizontalMoves > 0) {
				angle = 0;
			}
			else {
				angle = 180;
			}
			if(heading.equals("north")) {
				originalHeading = 90;
			}
			else {
				originalHeading = 270;				
			}
		}
		else {
			if(numOfVerticalMoves > 0) {
				angle = 90;
			}
			else {
				angle = 270;
			}
			
			if(heading.equals("west")) {
				originalHeading = 180;
			}
			else {
				originalHeading = 0;				
			}
		}
		
		// Circumvent code goes here.		
		while(!safe) {
			turnTo(angle, false);
			traverseATile();
			turnTo(originalHeading, false);
			safe = scanAheadForObstacle();
			if(safe) {
				traverseATile();
			}
		}

		
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
	
	public void turnRight(boolean wait) {
		setSpeeds(FAST,FAST);
		if(wait) {
			rotateBy(90,true);			
		}
		else {
			rotateBy(90,false);			
		}
	}
	
	public void turnLeft(boolean wait) {
		setSpeeds(FAST,FAST);
		if(wait) {
			rotateBy(-90,true);			
		}
		else {
			rotateBy(-90,false);			
		}
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
		
		/*
		for(int i = 0; i < restrictedAreaX.length; i++) {
			if(restrictedAreaX[i] == destX && restrictedAreaY[i] == destY) {
				return 2;
			}
		}
		*/
				
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
