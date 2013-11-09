import lejos.nxt.*;
import lejos.nxt.LCD;

/** Class that performs
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  1.0 November 2 2013
 */
public class LightLocalizer {

        private Odometer odo;
        private ColorSensor cs1;
        private ColorSensor cs2;
        private NXTRegulatedMotor leftMotor;
        private NXTRegulatedMotor rightMotor;
        private boolean localizing;
        
        /** Class constructor
         * 
         * @param s1 import ColorSensor from Port.1
         * @param s2 import ColorSensor from Port.2
         * @param odo import odometer to correct 
         */
        
        public LightLocalizer(ColorSensor cs1, ColorSensor cs2, NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor, Odometer odo){
                
        		/* Import the color sensors */
                this.cs1 = cs1;
                this.cs2 = cs2;
                
                /* Import the motor */
                this.leftMotor = leftMotor;
                this.rightMotor = rightMotor;
                this.odo = odo;
                
                /* Turn floodlights on */
                cs1.setFloodlight(2);
                cs2.setFloodlight(2);
                
        }
        
        /** Performs light localization and corrects odometer
         */
        public void doLocalization(){
                LCD.refresh();
                
                localizing = true;
                
                ColorSensor.Color color1 = cs1.getRawColor();
                ColorSensor.Color color2 = cs2.getRawColor();
                
                double blueLeft = color1.getBlue();
                double redLeft = color1.getRed();
                
                double blueRight = color2.getBlue();
                double redRight = color2.getRed();
                
                /* For testing purposes 
                LCD.drawString(Double.toString(blueLeft),0,1);
                LCD.drawString(Double.toString(green),0,2);
                LCD.drawString(Double.toString(blueRight),0,3);
                LCD.drawString(Double.toString(greenRight),0,4);
                */
            
                leftMotor.setSpeed(50);
                rightMotor.setSpeed(50);
                leftMotor.forward();
                rightMotor.forward();
                
                if (isLine(blueLeft, redLeft, 1) && isLine(blueRight, redRight,2)){
                        LCD.drawString("Both on the line!",0,1);
                }
                
                else if (isLine(blueLeft, redLeft,1)) {
                        LCD.drawString("left one         ", 0, 1);
                        do {
                        	leftMotor.stop();
                        } while (!(isLine(color2.getBlue(), color2.getRed(),2)));
                        leftMotor.setSpeed(50);
                        rightMotor.setSpeed(50);
                        leftMotor.forward();
                        rightMotor.forward();
                        
                }
                
                else if (isLine(blueRight, redRight,2)) {
                        LCD.drawString("right one        !",0,1);
                        do {
                        	leftMotor.stop();
                        } while (!(isLine(color1.getBlue(), color1.getRed(),1)));
                        leftMotor.setSpeed(50);
                        rightMotor.setSpeed(50);
                        leftMotor.forward();
                        rightMotor.forward();
                }
                else {
                        LCD.drawString("Not a line        ",0,1);
                }
        
        }
        
        /** Checks if the robot passes a line 
         * @param blue blue value read by the sensor 
         * @param red red value read by the sensor
         * @param line determines which sensor read the values
         * */
        public boolean isLine(double blue, double red, int line){
                /* Calibration for the right sensors's values
                 *  The values for the red are always 70 units higher than the value returned by the left sensor
                 */
                if (line==2){
                        red = red - 70;
                }
                
                /* If the values are within a threshold, return that it's a line **/
                if (blue<415 && blue>350 && red<475 && red>400) {
                        return true;  
                }
                
                /* It isn't a line, return false */
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
        
        /** 
         * 
         * @return the status of the robot's light localization. Returns true if the robot is localizing and false othewise
         */
        public boolean isLocalizing(){
                return this.localizing;
        }
        
}
