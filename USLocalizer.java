/*
 * Group 3
 * Simon Lei 260469075
 * Muhammad Hannan Aslam 260449459
 */

import lejos.nxt.*;

public class USLocalizer {
	public static int ROTATION_SPEED = 80;

	private Odometer odo;
	private UltrasonicSensor us;
	double angleA, angleB;
	
	private int WALL_DIST = 40;
	private int NOISE_MARGIN = 2;
	private double ORI_ANGLE;
	private boolean LOCALIZING;
	
	public static final double LEFT_RADIUS = 2.15;
	public static final double RIGHT_RADIUS = 2.15;
	public static final double WIDTH = 15.8;
	
	public USLocalizer(Odometer odo, UltrasonicSensor us) {
		this.odo = odo;
		this.us = us;
		Motor.A.setSpeed(ROTATION_SPEED);
		Motor.B.setSpeed(ROTATION_SPEED);
		
		// switch off the ultrasonic sensor
		us.off();
	}
	
	public void doLocalization() {
		LOCALIZING = true;
			
		// rotate the robot until it sees no wall
		this.rotateTillStopped (false);	
		this.detectWall (WALL_DIST, NOISE_MARGIN, false);
			
		// keep rotating until the robot sees a wall, then latch the angle
		this.rotateTillStopped (false);			
		this.detectWall (WALL_DIST, NOISE_MARGIN, true);
		angleA = odo.getAng();

		// switch direction and wait until it sees no wall
		this.rotateTillStopped (true);				
		this.detectWall (WALL_DIST, NOISE_MARGIN, false);
			
		// keep rotating until the robot sees a wall, then latch the angle
		this.rotateTillStopped (true);				
		this.detectWall (WALL_DIST, NOISE_MARGIN, true);		
		angleB = odo.getAng();
			
		// calculate required heading to correct odometer
		if (angleA < angleB) {
			ORI_ANGLE = 225 - (angleA + angleB)/2;
		}
		else {
			ORI_ANGLE = 45 - (angleA + angleB)/2;
		}

		// update the odometer position 
		odo.setPosition(new double [] {0.0, 0.0, odo.getAng() + ORI_ANGLE}, new boolean [] {true, true, true});	
	
		// orient at angle 90
		Motor.A.rotate(convertAngle(LEFT_RADIUS, WIDTH, (90 + (360 - odo.getAng()) % 360)), true);
		Motor.B.rotate(-convertAngle(RIGHT_RADIUS, WIDTH, (90 + (360 - odo.getAng()) % 360)), false);				
		Motor.A.stop();
		Motor.B.stop();
		LOCALIZING = false;
	}
	
	public void detectWall (int WALL_DIST, int NOISE_MARGIN, boolean LOOK_FOR_WALL) {
		while (Motor.A.isMoving()) {
			if (LOOK_FOR_WALL == true) {
				if (this.getFilteredData() < WALL_DIST - NOISE_MARGIN) {
					Motor.A.stop();
					Motor.B.stop();		
				}
			}
			else {
				if (this.getFilteredData() > WALL_DIST + NOISE_MARGIN) {
					try { Thread.sleep(1000); } catch (InterruptedException e) {};
					Motor.A.stop();
					Motor.B.stop();		
				}
			}
		}
	}
	
	public void rotateTillStopped (boolean CLOCKWISE) {
		if (CLOCKWISE == true) {
			Motor.A.rotate(convertAngle(LEFT_RADIUS, WIDTH, 360), true);
			Motor.B.rotate(-convertAngle(RIGHT_RADIUS, WIDTH, 360), true);	
		}
		else {
			Motor.A.rotate(-convertAngle(LEFT_RADIUS, WIDTH, 360), true);
			Motor.B.rotate(convertAngle(RIGHT_RADIUS, WIDTH, 360), true);	
		}
	}
	
	private int getFilteredData() {
		int distance;
		
		// do a ping
		us.ping();
		
		// wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		
		// there will be a delay here
		distance = us.getDistance();
			if (distance > 50)
				distance = 50;
		
		return distance;
	}
	
	// get and set methods for if the robot is localizing
	public void setIsLocalizing (boolean isLocalizing) {
		this.LOCALIZING = isLocalizing;
	}
	
	public boolean getIsLocalizing () {
		return this.LOCALIZING;
	}
	
	// Useful conversion methods taken from Lab2 SquareDriver.java file
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	private static int convertDistance(double radius, double vector_magnitude) {
		return (int) (( vector_magnitude * 180.0) / (Math.PI * radius));
	}
		
}
