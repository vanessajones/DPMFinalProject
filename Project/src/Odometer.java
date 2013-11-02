import lejos.util.Timer;
import lejos.util.TimerListener;
import lejos.nxt.NXTRegulatedMotor;

/** Class that update's the robot's position
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 *@version 1.0 November 2 2013
 */
public class Odometer implements TimerListener {

	private Timer timer;
	private NXTRegulatedMotor leftMotor, rightMotor;
	private final int DEFAULT_TIMEOUT_PERIOD = 20;
	private double leftRadius, rightRadius, width;
	private double x, y, theta;
	private double[] oldDH, dDH;
	
	/** Class constructor
	 * 
	 * @param leftMotor imports the left motor
	 * @param rightMotor imports the right motor
	 * @param INTERVAL ping interval
	 * @param autostart automatically start the timer
	 */
	public Odometer (NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor, int INTERVAL, boolean autostart) {
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		// default values, modify for your robot
		this.rightRadius = 2.18;
		this.leftRadius = 2.18;
		this.width = 15.8;
		
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 90.0;
		this.oldDH = new double[2];
		this.dDH = new double[2];

		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout 
			this.timer = new Timer((INTERVAL <= 0) ? INTERVAL : DEFAULT_TIMEOUT_PERIOD, this);
			this.timer.start();
		} else
			this.timer = null;
	}
	
	/** Method to stop the timerListener
	 * 
	 */
	public void stop() {
		if (this.timer != null)
			this.timer.stop();
	}
	
	/** Method to start the timerListener
	 * 
	 */
	public void start() {
		if (this.timer != null)
			this.timer.start();
	}
	
	/** Calculated the displacement and heading of the robot
	 * 
	 * @param data 
	 */
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI / 360.0;
		data[1] = (rightTacho * rightRadius - leftTacho * leftRadius) / width;
	}
	
	/*
	 * Recompute the odometer values using the displacement and heading changes
	 */
	public void timedOut() {
		this.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];

		// update the position in a critical region
		synchronized (this) {
			theta += dDH[1];
			theta = fixDegAngle(theta);

			x += dDH[0] * Math.cos(Math.toRadians(theta));
			y += dDH[0] * Math.sin(Math.toRadians(theta));
		}

		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}

	/** Returns the position of the robot
	 * 
	 * @return returns the x-position
	 */
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	/** Returns the position of the robot
	 * 
	 * @return returns the y-position
	 */
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	/** Returns the heading of the robot
	 * 
	 * @return the heading of the robot, in degrees 
	 */
	public double getAng() {
		synchronized (this) {
			return theta;
		}
	}

	/** Set the robot's position
	 * 
	 * @param position update the robot's position
	 * @param update determines which values need to be updated [x-position, y-position, angle]
	 */
	public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/** Returns the robot's position in an array
	 * 
	 * @param position fills the position array with the robot's current position
	 */
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	/** Return an array with the robot's position
	 * 
	 * @return an array with the robot's position
	 */
	public double[] getPosition() {
		synchronized (this) {
			return new double[] { x, y, theta };
		}
	}
	
	/** Access the robot's motors for movement (left motor and right motor)
	 * 
	 * @return the robot's motors
	 */
	public NXTRegulatedMotor [] getMotors() {
		return new NXTRegulatedMotor[] {this.leftMotor, this.rightMotor};
	}
	
	/** Access the left motor
	 * 
	 * @return the left motor
	 */
	
	public NXTRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}
	
	/** Access the right motor
	 * 
	 * @return the right motot
	 */
	public NXTRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	/** Fix the angle: makes an angle positive and smaller than 360 degrees
	 * 
	 * @param angle angle that needs to be fixed
	 * @return angle within [0,360] degrees
	 */
	public static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}
	
	/** Calculate the minimum angle between two headings
	 * 
	 * @param a first angle
	 * @param b second angle
	 * @return minimum angle between the angle a and b
	 */
	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);

		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}
}
