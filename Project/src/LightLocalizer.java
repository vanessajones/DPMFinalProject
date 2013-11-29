import lejos.nxt.*;

/** Class that performs light correction using two light sensors
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  3.0 November 28 2013
 */
public class LightLocalizer {

        private Odometer odo;
        private ColorSensor cs1;
        private ColorSensor cs2;
        private NXTRegulatedMotor leftMotor;
        private NXTRegulatedMotor rightMotor;
        private boolean localizing;
        /**
         * angle 
         */
        private final double angleThreshold = 40;  		
        private final double sensorCorrection = 10;		
        private final double lightThreshold = 485;	
        
        /** Class constructor
         * 
         * @param cs1 import ColorSensor from Port.1
         * @param cs2 import ColorSensor from Port.2
         * @param odo import odometer to correct 
         * @param leftMotor import left motor to move the left wheel
         * @param righMotor import right motor to move the right wheel
         */
        public LightLocalizer(ColorSensor cs1, ColorSensor cs2, NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor, Odometer odo){
                
                this.cs1 = cs1;
                this.cs2 = cs2;
                
                this.leftMotor = leftMotor;
                this.rightMotor = rightMotor;
                
                this.odo = odo;
                
                /* Turn floodlights on to the color (default is red) */
                cs1.setFloodlight(true);
                cs2.setFloodlight(true);
                
        }
        
        /** Performs light localization as the robot travels over a gridline and corrects odometer
         */
        public void doLocalization(){
                
        		// set localizing to true
                localizing = true;
                
                // Store the normalized value of the brightness of the white light detected, such that the lowest value is darkness
                // and the highest value is intense bright light.
                double left = cs1.getNormalizedLightValue();
                double right =  cs2.getNormalizedLightValue();
                
                // set the motors speed to 70, and make the robot go forward
                leftMotor.setSpeed(70);
                rightMotor.setSpeed(70);
                leftMotor.forward();
                rightMotor.forward();
                
                /* If both the robot's wheels are on the gridline, the robot is going straight, 
                 * thus, only correct the robot's odometer
                 */
                if (isLine(left, 1) && isLine(right, 2)){
            
                	updateOdo();
             
                }
                
                /* If only the robot's left wheel is over a gridline, stop (almost) the left wheel until the right wheel catches up
                 * Afterwards, the robot's heading will be corrected and the robot's odometer is updated.
                 */
                else if (isLine(left,1)) {
                       // while the right wheel doesn't detect a line, keep the left wheel stopped
                        do {
                        	leftMotor.setSpeed(10);
                        } while (!(isLine(cs2.getNormalizedLightValue(),2)));
                        
                        // set the robot's speeds again to continue in a straight heading
                        leftMotor.setSpeed(50);
                        rightMotor.setSpeed(50);
                        leftMotor.forward();
                        rightMotor.forward();
                        
                        // update the odometry
                        updateOdo();
                }
                
                /* If only the robot's right wheel is over a gridline, stop (almost) the right wheel until the left wheel catches up
                 * Afterwards, the robot's heading will be corrected and the robot's odometer is updated.
                 */
                else if (isLine(right,2)) {
                	 // while the left wheel doesn't detect a line, keep the right wheel stopped
                        do {
                        	rightMotor.setSpeed(10);
                        } while (!(isLine(cs1.getNormalizedLightValue(),1)));
                        
                        // set the robot's speeds again to continue in a straight heading
                        leftMotor.setSpeed(50);
                        rightMotor.setSpeed(50);
                        leftMotor.forward();
                        rightMotor.forward();
                        
                        // update the odometry
                        updateOdo();
                }
              
                /* If both light sensors did not detect any gridlines, wait and perform localization again
                 */
                else {
                	 try{ Thread.sleep(300);
                     } catch (Exception e){}
                       doLocalization();
                }    
                
        }

        
        
        /** Determines if the robot is detecting a grid line
         * @param light normalized light value returned by the light sensor
         * @param line determines which sensor read the values (for calibration purposes)
         * */
        public boolean isLine(double light, int line){
                /* Calibration for the right sensors's values
                 *  The values for the red are always sensorCorrection units higher than the value returned by the left sensor
                 */
                if (line==2){
                        light = light - sensorCorrection;
                }
                
                /* If the values are under a certain threshold, the robot has passed over a gridline.
                 * Return true, it is a line */
                if (light < lightThreshold ) {
                        return true;  
                }
                
                /* It is not a line, return false */
                else {
                        return false; 
                }
        }
        
        
        /** Sets the robot to localizing/not localizing
         * @param isLocalizing sets the robot's status. 
         */
        
        public void setLocalizing(boolean isLocalizing){
                this.localizing = isLocalizing;
        }
        
        /** Updates the odometer, after going over a line 
         * 
         */
        public void updateOdo(){
        	// get the x and y position read by the odometry
        	double x = odo.getX();
        	double y = odo.getY();
        	
        	double line;
        	double position; 
        	
        	// if the robot is (increasingly) going along the y-direction, update the y-position and the heading
        	if ((odo.getAng()>90-angleThreshold && odo.getAng()<90+angleThreshold)){
        		// determine which line the robot has crossed by dividing the y-position returned by the odometer
        		line = (int)((y + 15) / 30); 
        		// multiply by the length of a tile to know the y-position
        		position = line*30;
        		odo.setPosition(new double [] {0.0, position , 90}, new boolean [] {false, true, true});	
        	}
        	
        	// if the robot is going (decreasing) along the y-direction, update the y-position and the heading
        	else if (odo.getAng()>270-angleThreshold && odo.getAng()<270+angleThreshold){
        		// determine which line the robot has crossed by dividing the y-position returned by the odometer
        		line = (int)((y + 15) / 30); 
        		// multiply by the length of a tile to know the y-position
        		position = line*30;
        		odo.setPosition(new double [] {0.0, position , 270}, new boolean [] {false, true, true});	
        	}
        	
        	// if the robot is going (decreasing) along the x-direction, update the x-position and the heading
        	else if (odo.getAng()>180-angleThreshold && odo.getAng()<180+angleThreshold) {
        		// determine which line the robot has crossed by dividing the y-position returned by the odometer
        		line = (int)((x + 15) / 30); 
        		// multiply by the length of a tile to know the y-position
        		position = line*30;
        		odo.setPosition(new double [] {position, 0.0 , 180}, new boolean [] {true, false, true});	
        	}
        	
        	// if the robot is going (increasing) along the x-direction, update the x-position and the heading
        	else {
        		line = (int)((x + 15) / 30); 
        		position = line*30;
        		odo.setPosition(new double [] {position, 0.0 , 0}, new boolean [] {true, false, true});	
        	}
        	
        }
        
        /** Determines if the robot is perfoming light localization 
         * 
         * @return the status of the robot's light localization. Returns true if the robot is localizing and false othewise.
         */
        public boolean isLocalizing(){
                return this.localizing;
        }
        
        
}
