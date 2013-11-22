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
import lejos.nxt.comm.RConsole;

import java.lang.Math;


/** Class that houses the commands related to navigation.
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  1.0 November 2 2013
 */

public class Navigation {
	
	final static int VERY_FAST = 260, FAST = 200, SLOW = 140, ACCELERATION = 4000;
	final static double DEG_ERR = 3.0, CM_ERR = 1.0;
	
	private Odometer odometer;
	private NXTRegulatedMotor leftMotor, rightMotor;
	private LightLocalizer liLocalizer;
	private ObjectDetection objDetection;
	private HandleBlock handle;
	
	private final double RADIUS = 2.2;
	private final double WIDTH = 20;
	private final double TILELENGTH = 30;
	private final int ERRORMARGIN = 10;
	
	private boolean hasBlock = false;
	private boolean foundBlock = false;
	private boolean justWentStraight = true;
	
	private final int leftWallBound;
	private final int rightWallBound;
	private final int topWallBound;
	private final int bottomWallBound;
	private int[] restrictedAreaX;
	private int[] restrictedAreaY;
	
	private int[] dropzoneX;
	private int[] dropzoneY;
	
	private int closestDropZonePtX;
	private int closestDropZonePtY;
	private int numOfHorizontalMoves;
	private int numOfVerticalMoves;
	private int destX;
	private int destY;
		
	/** Class constructor
	 * 
	 * @param odo imports Odometer 
	 * @param ls imports LightLocalizer
	 * @param od imports ObjectDetection
	 */
	
	public Navigation(Odometer odo, ObjectDetection od, LightLocalizer ls, HandleBlock hb, int role, int[] greenZone, int[] redZone) {
		this.odometer = odo;
		this.liLocalizer = ls;
		this.objDetection = od;
		this.handle = hb;
		
		this.leftMotor = Motor.A;
		this.rightMotor = Motor.B;

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
		
		leftWallBound = -25;
		rightWallBound = 325;
		topWallBound = 325;
		bottomWallBound = -25;
		
		interpretBluetooth(greenZone, redZone, role);
	}
	
	// A method that interprets bluetooth values into values good for navigation
	  
	  public void interpretBluetooth(int[] greenZone, int[] redZone, int role) {
		  int numOfRedPts = ((redZone[2]-redZone[0]) + (redZone[3]-redZone[1]))*2;
		  
	  		if(role == 1) {
	 			this.dropzoneX = new int[4];
	  			this.dropzoneY = new int[4];
	  			
	  			dropzoneX[0] = greenZone[0]*30;
	  			dropzoneY[0] = greenZone[1]*30;
	  			dropzoneX[1] = (greenZone[0] + 1)*30;
	 			dropzoneY[1] = greenZone[1]*30;
	 			dropzoneX[2] = greenZone[0]*30;
	  			dropzoneY[2] = (greenZone[1] + 1)*30;
	 			dropzoneX[3] = (greenZone[0] + 1)*30;
	  			dropzoneY[3] = (greenZone[1] + 1)*30;
	  
	  			this.restrictedAreaX = new int[numOfRedPts];
	  			this.restrictedAreaY = new int[numOfRedPts];
	  			
	  			restrictedAreaX[0] = redZone[0]*30;
	  			restrictedAreaY[0] = redZone[1]*30;
	  			restrictedAreaX[1] = redZone[2]*30;
	  			restrictedAreaY[1] = redZone[1]*30;
	  			restrictedAreaX[2] = redZone[0]*30;
	  			restrictedAreaY[2] = redZone[3]*30;
	  			restrictedAreaX[3] = redZone[2]*30;
	  			restrictedAreaY[3] = redZone[3]*30;
	  			
	  			int index = 4;
	  			
	  			for(int i = redZone[0]+1; i < redZone[2]; i++) {
	  				restrictedAreaX[index] = redZone[i]*30;
	  				restrictedAreaY[index] = redZone[1]*30;
	  				restrictedAreaX[index+1] = redZone[i]*30;
	  				restrictedAreaY[index+1] = redZone[3]*30;
	  				index = index + 2;
	  			}
	  			
	  			for(int j = redZone[1]+1; j < redZone[3]; j++) {
	  				restrictedAreaX[index] = redZone[0]*30;
	  				restrictedAreaY[index] = redZone[j]*30;
	  				restrictedAreaX[index+1] = redZone[2]*30;
	  				restrictedAreaY[index+1] = redZone[j]*30;
	  				index = index + 2;	  				
	  			}
	  		}
	  
	  		else if(role == 2) {
	  			this.dropzoneX = new int[numOfRedPts];
	  			this.dropzoneY = new int[numOfRedPts];
	  			
	  			dropzoneX[0] = redZone[0]*30;
	  			dropzoneY[0] = redZone[1]*30;
	  			dropzoneX[1] = redZone[2]*30;
	  			dropzoneY[1] = redZone[1]*30;
	  			dropzoneX[2] = redZone[0]*30;
	  			dropzoneY[2] = redZone[3]*30;
	  			dropzoneX[3] = redZone[2]*30;
	  			dropzoneY[3] = redZone[3]*30;
	  			
	  			int index = 4;
	  			
	  			for(int i = redZone[0]+1; i < redZone[2]; i++) {
	  				dropzoneX[index] = redZone[i]*30;
	  				dropzoneY[index] = redZone[1]*30;
	  				dropzoneX[index+1] = redZone[i]*30;
	  				dropzoneY[index+1] = redZone[3]*30;
	  				index = index + 2;
	  			}
	  			
	  			for(int j = redZone[1]+1; j < redZone[3]; j++) {
	  				dropzoneX[index] = redZone[0]*30;
	  				dropzoneY[index] = redZone[j]*30;
	  				dropzoneX[index+1] = redZone[2]*30;
	  				dropzoneY[index+1] = redZone[j]*30;
	  				index = index + 2;	  				
	  			}
	  			
	  			int numOfGreenPts = ((greenZone[2]-greenZone[0]) + (greenZone[3]-greenZone[1]))*2;
	  			
	  			this.restrictedAreaX = new int[numOfRedPts];
	  			this.restrictedAreaY = new int[numOfRedPts];
	  			
	  			restrictedAreaX[0] = greenZone[0]*30;
	  			restrictedAreaY[0] = greenZone[1]*30;
	  			restrictedAreaX[1] = greenZone[2]*30;
	  			restrictedAreaY[1] = greenZone[1]*30;
	  			restrictedAreaX[2] = greenZone[0]*30;
	  			restrictedAreaY[2] = greenZone[3]*30;
	  			restrictedAreaX[3] = greenZone[2]*30;
	  			restrictedAreaY[3] = greenZone[3]*30;
	  			
	  			index = 4;
	  			
	  			for(int i = greenZone[0]+1; i < greenZone[2]; i++) {
	  				restrictedAreaX[index] = greenZone[i]*30;
	  				restrictedAreaY[index] = greenZone[1]*30;
	  				restrictedAreaX[index+1] = greenZone[i]*30;
	  				restrictedAreaY[index+1] = greenZone[3]*30;
	  				index = index + 2;
	  			}
	  			
	  			for(int j = greenZone[1]+1; j < greenZone[3]; j++) {
	  				restrictedAreaX[index] = greenZone[0]*30;
	  				restrictedAreaY[index] = greenZone[j]*30;
	  				restrictedAreaX[index+1] = greenZone[2]*30;
	  				restrictedAreaY[index+1] = greenZone[j]*30;
	  				index = index + 2;	  				
	  			}
	  		}
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
						foundBlock = true;
						goGrabBlock(coords);
						leftMotor.stop();
						rightMotor.stop();
						
					} catch (Exception e) {
						// no blue block found, keep going
					}
				}
				if(!foundBlock) {
					pickSafeRoute();				
					traverseATile();
				}
			}
			
			else {
				turnLeft(false);
				
				while(leftMotor.isMoving() || rightMotor.isMoving()) {
					try {
						coords = objDetection.findObject();
						Sound.beep();
						foundBlock = true;
						goGrabBlock(coords);
						leftMotor.stop();
						rightMotor.stop();
						
					} catch (Exception e) {
						// no blue block found, keep going
					}
				}
				if(!foundBlock) {
					pickSafeRoute();				
					traverseATile();
				}
			}
		}
		
	}
	
	public boolean scanAheadForObstacle() {
		double originalAngle = odometer.getAng();
		setSpeeds(FAST,FAST);

		rotateBy(15,false);
		while(leftMotor.isMoving() || rightMotor.isMoving()) {
			if(objDetection.isObstacle()) {
				turnTo(originalAngle, false, FAST);
				return false;
			}
		}
		
		rotateBy(-30,false);
		while(leftMotor.isMoving() || rightMotor.isMoving()) {
			if(objDetection.isObstacle()) {
				turnTo(originalAngle, false, FAST);
				return false;
			}
		}
		
		turnTo(originalAngle,false, FAST);
		return true;
	}
	
	/**
	 * Looks for a line intersection that isn't blocked or out of bounds.
	 */

	public void pickSafeRoute() {
		boolean notSafe;
		int order;
		
		getNextLineIntersection();
	    order = isBoundary(destX,destY);
	    
	    if(order == 0 && scanAheadForObstacle()) {
	    	justWentStraight = !justWentStraight;
	    }
		
	    else {
	    	notSafe = true;
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
		setSpeeds(VERY_FAST,VERY_FAST);
		travelBy((TILELENGTH - 5),true);
		liLocalizer.doLocalization();
	}
	
	
	/**
	 * Moves the robot to the detected blue block, robot will travel to inputs after it's grabbed the block
	 */
	public void goGrabBlock(double[] coords) {
		double[] currentPos = new double[] {odometer.getX(),odometer.getY(),odometer.getAng()};
		setSpeeds(FAST,FAST);
		//LCD.drawInt((int)coords[0]*100, 0, 0);
		//LCD.drawInt((int)coords[1]*100, 0, 1);
		while (objDetection.getFilteredData() > 5 && (leftMotor.isMoving() || rightMotor.isMoving())){
			travelTo(coords[0],coords[1]);
		}
		setSpeeds(FAST,FAST);
		travelTo(currentPos[0],currentPos[1]);
		setSpeeds(FAST,FAST);
		rotateBy(20,true);
		rotateBy(-40,true);
		rotateBy(40,true);
		rotateBy(-20, true);
		travelBy(-2,true);
		handle.capture();
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
			if((dropzoneX[i] - odometer.getX()) >= 0) {
				pathLengthX = (int)((dropzoneX[i] - odometer.getX() + ERRORMARGIN) / TILELENGTH);
			}
			else {
				pathLengthX = (int)((dropzoneX[i] - odometer.getX() - ERRORMARGIN) / TILELENGTH);				
			}
			
			if((dropzoneY[i] - odometer.getY()) >= 0) {
				pathLengthY = (int)((dropzoneY[i] - odometer.getY() + ERRORMARGIN) / TILELENGTH);
			}
			else {
				pathLengthY = (int)((dropzoneY[i] - odometer.getY() - ERRORMARGIN) / TILELENGTH);				
			}
			
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
	public void bringToDropZone() {
		int order;
		int xPos = (int)odometer.getX(), yPos = (int)odometer.getY(), ang = (int)odometer.getAng() ;
		Boolean isSafe;
		getShortestPathToDropZone();
		
		while (numOfVerticalMoves != 0 || numOfHorizontalMoves != 0) {
			isSafe = true;
			
			if(Math.abs(numOfVerticalMoves) >= Math.abs(numOfHorizontalMoves)) {
				if(numOfVerticalMoves > 0) {
					turnTo(90,false, VERY_FAST);
				}
				else {
					turnTo(270,false, VERY_FAST);
				}
			}
			else {
				if(numOfHorizontalMoves > 0) {
					turnTo(0,false, VERY_FAST);
				}
				else {
					turnTo(180,false, VERY_FAST);
				}
			}
			
			// call obstacle detection, if object in way, change safe to false
			getNextLineIntersection();
			order = isBoundary(destX,destY);
			if(!scanAheadForObstacle()) {
				isSafe = false;
			}
			if(order != 0) {
				isSafe = false;
			}
			if(isSafe) {
				if((Math.abs(numOfVerticalMoves) == 1 && numOfHorizontalMoves == 0) || (numOfVerticalMoves == 0 && Math.abs(numOfHorizontalMoves) == 1) || (numOfVerticalMoves == 0 && numOfHorizontalMoves == 0)) {
					
					// where the robot will go after its placed the block at drop zone, these coordinates are convenient because
					// the robot's already traveled through here, thus we know there's no obstacles.

					xPos = (int)odometer.getX();
					yPos = (int)odometer.getY();
					ang = (int)((odometer.getAng() + 180) % 360);
					setSpeeds(0,0);
					handle.lift();

				}
				setSpeeds(VERY_FAST,VERY_FAST);
				traverseATile();
				getShortestPathToDropZone();
							
			}
			// robot moves past obstacle and updates shortest path
			else {
				circumventObstacle();
				getShortestPathToDropZone();
				
			}
		}
			
		if(closestDropZonePtX == dropzoneX[0] && closestDropZonePtY == dropzoneY[0]) {
			turnTo(45,true, SLOW);
		}
		
		else if(closestDropZonePtX == dropzoneX[1] && closestDropZonePtY == dropzoneY[1]) {
			turnTo(135, true, SLOW);
		}
		else if(closestDropZonePtX == dropzoneX[2] && closestDropZonePtY == dropzoneY[2]) {
			turnTo(315, true, SLOW);
		}
		else if(closestDropZonePtX == dropzoneX[3] && closestDropZonePtY == dropzoneY[3]) {
			turnTo(225, true, SLOW);
		}

		setSpeeds(SLOW,SLOW);
		travelBy(9,true);
		handle.lower();
		setSpeeds(VERY_FAST,VERY_FAST);
		travelBy(-24,true);
		travelTo(xPos,yPos);
		turnTo(ang,true,VERY_FAST);
		foundBlock= false;
	}
		
	public void circumventObstacle() {
		String heading = getRobotDirection();
		int angle;
		int originalHeading;
		boolean safe = false;
		Sound.buzz();
		
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
			turnTo(angle, false, VERY_FAST);
			traverseATile();
			turnTo(originalHeading, false, VERY_FAST);
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
			this.turnTo(minAng, false, SLOW);
			this.setSpeeds(VERY_FAST, VERY_FAST);
		}
		this.setSpeeds(0, 0);
	}

	/**
	 * Tells robot to rotate to a specific angle.
	 * @param angle the angle the robot will rotate to.
	 * @param stop controls whether or not to stop the motors when the turn is completed.
	 */
	public void turnTo(double angle, boolean stop, int speed) {
		speed = speed;
		double error = angle - this.odometer.getAng();

		while (Math.abs(error) > DEG_ERR) {

			error = angle - this.odometer.getAng();

			if (error < -180.0) {
				this.setSpeeds(-speed, speed);
			} else if (error < 0.0) {
				this.setSpeeds(speed, -speed);
			} else if (error > 180.0) {
				this.setSpeeds(speed, -speed);
			} else {
				this.setSpeeds(-speed, speed);
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
		
		
		for(int i = 0; i < restrictedAreaX.length; i++) {
			if(restrictedAreaX[i] == destX && restrictedAreaY[i] == destY) {
				return 2;
			}
		}			
		
		if(!foundBlock) {
			for(int i = 0; i < dropzoneX.length; i++) {
				if(dropzoneX[i] == destX && dropzoneY[i] == destY) {
					return 2;
				}
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
