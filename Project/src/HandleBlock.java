import lejos.nxt.*;

/** Class that manipulates the block with one motor
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  1.0 November 2 2013
 */
public class HandleBlock {

	private final double CAPTURE_DEGREES = 100; // tentative value, must test it
	private final double LIFT_DEGREES = 100; // tentative value, must test it
	private boolean isManipulating; // boolean that determines if the block is currently being manipulated
	private NXTRegulatedMotor lifting;

	/** Constructor method
	 *  
	 */
	public HandleBlock(){
		// use the Motor.C for the block lifting mechanism
		this.lifting = Motor.C;
	}
	
	/** Captures the blue styrofoam block by rotating the motor
	 * 
	 */
	public void capture(){
		
	}
	
	/** After capturing the blue styrofoam block, the motor is rotated a certain amount of degrees to lift the block.
	 * 
	 */
	public void lift(){
	}
	
	/** Lower the block to stack
	 * 
	 */
	public void lower(){
	}
	
	/** Release the block
	 * 
	 */
	public void release() {
	}
	
	/** Getter that determines if the robot is handling a block
	 * 
	 * @return determines if the robot is handling a block. Return true if the block is handling a block and false if its not.
	 */
	public boolean isHandling(){
		return true;
	}
	
	/** Setter that sets the robot to handling block
	 * 
	 */
	public void setHandling(){
		
	}
}
