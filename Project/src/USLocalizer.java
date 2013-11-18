import lejos.nxt.*;

/** This class performs localization with the Ultrasonic Sensor
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version 1.0 November 2 2013
 *  
 */
public class USLocalizer {

	private Odometer odo;
	private UltrasonicSensor us;
	
	public double angleA, angleB;
	private int WALL_DIST = 40;
	private int NOISE_MARGIN = 2;
	private double ORI_ANGLE;
	private boolean LOCALIZING;

	public static int ROTATION_SPEED = 150;
	public static final double LEFT_RADIUS = 2.15;
	public static final double RIGHT_RADIUS = 2.15;
	public static final double WIDTH = 15.8;
	
	/**
	 * Class constructor
	 * @param odo imports the Odometer
	 * @param us imports one UltrasonicSensor
	 */
	public USLocalizer(Odometer odo, UltrasonicSensor us) {
		this.odo = odo;
		this.us = us;
		Motor.A.setSpeed(ROTATION_SPEED);
		Motor.B.setSpeed(ROTATION_SPEED);
		
		// switch off the ultrasonic sensor
		us.off();
	}
	
	/** doLocalization method that makes the robot perform Falling Edge localization to correct its heading
	 * and its position
	 */
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
	

//		LCD.drawString(Double.toString(ORI_ANGLE+odo.getAng()),0,6);
		// orient at angle 90
	
		
	/* 	
		// update the odometer's position according to the starting position
		
		if (startingPosition == X2){
			odo.setPosition(new double [] {304.8, 0, 270}, new boolean [] {true, false, true});
		}
		
		else if (startingPosition == X3){
			odo.setPosition(new double [] {304.8, 304.8, 180 }, new boolean [] {true, true, true});
			}
		
		else if (startingPosition == X4){
			odo.setPosition(new double [] {0, 304.8, 90}, new boolean [] {false, true, true});
	*/
		LOCALIZING = false;
	}
	
	/** Method that detects the wall 
	 * 
	 * @param WALL_DIST wall filter
	 * @param NOISE_MARGIN noise filter
	 * @param LOOK_FOR_WALL boolean that determines if the robot must look for the wall
	 */
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
	
	/** rotateTillStopped method that makes the robot rotate 
	 * @param CLOCKWISE determines if the robot turns clockwise or counterclockwise
	 * 
	 */
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
	
	/** Method that implements a filter for the data from the Ultrasonic Sensor
	 * @return distance filtered distance read from the ultrasonic sensor
	 */
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
	

	/** setter method to set the robot as localizing or not
	 * 
	 * @param isLocalizing boolean that indicates the status of localization. True if the robot is
	 * still localizing and false is the robot is not localizing
	 */
	public void setIsLocalizing (boolean isLocalizing) {
		this.LOCALIZING = isLocalizing;
	}
	
	/** getter method that returns the status of the robot's localization
	 * @return status of the robot's localization, true if the robot is localizing and 
	 * false is the robot is not localizing
	 */
	public boolean getIsLocalizing () {
		return this.LOCALIZING;
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
