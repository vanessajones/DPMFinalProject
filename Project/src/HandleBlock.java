		import lejos.nxt.*;
		
		/** Class that manipulates the block with one motor
		 * 
		 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
		 * @version  1.0 November 2 2013
		 */
		public class HandleBlock {
		
			private final int CAPTURE_DEGREES = -250; // tentative value, must test it
			private final int LIFT_DEGREES = -450; // tentative value, must test it
			private final int LOWER_DEGREES = 0;
			private final int RELEASE_DEGREES = 20;
			private boolean manipulating; // boolean that determines if the block is currently being manipulated
			private NXTRegulatedMotor liftingMotor;
		
	
			        /** Constructor method
			         *  
			         */
			        public HandleBlock(NXTRegulatedMotor lifting){
			                // use the Motor.C for the block lifting mechanism
			                this.liftingMotor = lifting;
			                liftingMotor.setSpeed(100);
			                
			        }
			        
			        /** Put the claw back at position 0. In case the claw is being weird, we can always call this function to set
			         * it back to its initial position 
			         * Tested --> successful
			         */
			        public void initialPosition(){
			                liftingMotor.rotateTo(0);
			        }
			        
			        /** Captures the blue styrofoam block by rotating the motor
			         * 
			         */
			        public void capture(){
			        
			                liftingMotor.rotateTo(CAPTURE_DEGREES);
			        }
			        
			        /** After capturing the blue styrofoam block, the motor is rotated a certain amount of degrees to lift the block.
			         * Tested --> Successful
			         */
			        public void lift(){
			        		
			                liftingMotor.rotateTo(LIFT_DEGREES);
			        }
			        
			        /** Release the block the block to stack
			         * 
			         */
			        public void lower(){
			                liftingMotor.rotateTo(0);
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

