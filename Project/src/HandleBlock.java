import lejos.nxt.*;

/** Class that manipulates the block with one motor
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  3.0 November 28 2013
 */
public class HandleBlock {

	private final int CAPTURE_DEGREES = -250; 
	private final int LIFT_DEGREES = -450; 
	private final int LOWER_DEGREES = 0;
	private final int RELEASE_DEGREES = 20;
	private boolean manipulating; // boolean that determines if the block is currently being manipulated
	private NXTRegulatedMotor liftingMotor;

	/***
	 * Class contructor
	 * @param NXTRegulatedMotor imports the motor used for moving the mechanism (claw)
	 */
	public HandleBlock(NXTRegulatedMotor lifting){
		// use the Motor.C for the block lifting mechanism
		this.liftingMotor = lifting;
		// set the speed
		liftingMotor.setSpeed(100);

	}

	/** Moves the claw back to its initial position.
	 * 
	 */
	public void initialPosition(){
		// in this case, the initial position is the arms fully opened
		liftingMotor.rotateTo(0);
	}

	/** Captures the blue styrofoam block by rotating the motor
	 * 
	 */
	public void capture(){

		liftingMotor.rotateTo(CAPTURE_DEGREES);
	}

	/** Lifts the blue styrofoam block by rotating the motor
	 * 
	 */
	public void lift(){

		liftingMotor.rotateTo(LIFT_DEGREES);
	}

	/** Lowers the claw and releases the block by rotating to the claw initial position
	 * 
	 */
	public void lower(){
		liftingMotor.rotateTo(0);
	}


	/** Determines if the robot is handling a block
	 * @return determines if the robot is handling a block. Returns true if the block is handling a block and false if its not
	 */
	public boolean isHandling(){
		return this.manipulating;
	}

	/** Sets the robot to handling a block
	 * @param handling set if the robot is manipulating a block or not. Set to true if the robot is manipulating a block and false otherwise
	 */
	public void setHandling(boolean handling){
		this.manipulating = handling;
	}
}

