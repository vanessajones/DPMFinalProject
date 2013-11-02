import lejos.nxt.*;

/** Class that performs
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  1.0 November 2 2013
 */
public class LightLocalizer {

	private Odometer odo;
	private ColorSensor cs1;
	private ColorSensor cs2;
	
	private boolean localizing;
	
	/** Class constructor
	 * 
	 * @param s1 import ColorSensor from Port.1
	 * @param s2 import ColorSensor from Port.2
	 * @param odo import odometer to correct 
	 */
	
	public LightLocalizer(ColorSensor s1, ColorSensor s2, Odometer odo){
		this.cs1 = cs1;
		this.cs2 = cs2;
		this.odo = odo;
	}
	
	/** Performs light localization and corrects odometer
	 */
	public void doLocalization(boolean localizing){
		/*
		 * Put code for localization here
		 */
	}
	
	/** Sets the robot to localizing/not localizing
	 * @param isLocalizing sets the robot's status. 
	 */
	public void setLocalizing(boolean isLocalizing){
		this.localizing = isLocalizing;
	}
	
	/** 
	 * 
	 * @return the status of the robot's light localization. Returns true if the robot is localizing and false othewise
	 */
	public boolean isLocalizing(){
		return this.localizing;
	}
	
}
