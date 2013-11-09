import lejos.nxt.*;

/** Class that manipulates the block with one motor
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  2.0 November 6 2013
 */
public class HandleBlock {

	private final double CAPTURE_DEGREES = 100; // tentative value, must test it
	private final double LIFT_DEGREES = 100; // tentative value, must test it
	private boolean manipulating; // boolean that determines if the block is currently being manipulated
	private NXTRegulatedMotor liftingMotor;

	/** Constructor method
	 *  
	 */
	public HandleBlock(){
		// use the Motor.C for the block lifting mechanism
		this.liftingMotor = Motor.C;
	}
	
	/** Put the claw back at position 0. In case the claw is being weird, we can always call this function to set
	 * it back to its initial position 
	 * Tested --> successful
	 */
	public void initialPosition(){
		liftingMotor.rotateTo(0, true);
	}
	
	/** Captures the blue styrofoam block by rotating the motor
	 * 
	 */
	public void capture(){
		liftingMotor.rotate(0);
	}
	
	/** After capturing the blue styrofoam block, the motor is rotated a certain amount of degrees to lift the block.
	 * Tested --> Successful
	 */
	public void lift(){
		liftingMotor.rotateTo(-350);
	}
	
	/** Lower the block to stack
	 * 
	 */
	public void lower(){
		liftingMotor.rotateTo(0);
	}
	
	/** Release the block
	 *  Tested but the angle is not completely right yet.
	 */
	public void release() {
		liftingMotor.rotateTo(20);
	}
	
	/** Getter that determines if the robot is handling a block
	 * @return determines if the robot is handling a block. Return true if the block is handling a block and false if its not.
	 */
	public boolean isHandling(){
		return this.manipulating;
	}
	
	/** Setter that sets the robot to handling block
	 * @param handling set if the robot is manipulating a block. True if the robot is manipulating it and false otherwise
	 */
	public void setHandling(boolean handling){
		this.manipulating = handling;
	}
}
