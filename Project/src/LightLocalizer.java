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
        private final double angleThreshold = 3;  		// test this
        private final double sensorCorrection = 70;		// test this
        private final double lightThreshold = 480;		// test this 
        
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
                cs1.setFloodlight(true);
                cs2.setFloodlight(true);
                
        }
        
        /** Performs light localization and corrects odometer
         */
        public void doLocalization(){
                LCD.refresh();
                
                localizing = true;
                
                double left = cs1.getNormalizedLightValue();
                double right =  cs2.getNormalizedLightValue();
                
                LCD.drawString(Double.toString(left),0,1);
                LCD.drawString(Double.toString(right),0,2);
          
                /*
                leftMotor.setSpeed(50);
                rightMotor.setSpeed(50);
                leftMotor.forward();
                rightMotor.forward();
                */
                
                if (isLine(left, 1) && isLine(right, 2)){
                	
                		/* for testing only
                        LCD.drawString("Both on the line!",0,1);
                        
                        LCD.drawString(Double.toString(odo.getX()),0,1);
                        LCD.drawString(Double.toString(odo.getY()),0,2);
                        LCD.drawString(Double.toString(odo.getAng()),0,3);
                        */
                       
                        updateOdo();
                        
                        /* for testing only
                        LCD.drawString(Double.toString(odo.getX()),0,1);
                        LCD.drawString(Double.toString(odo.getY()),0,2);
                        LCD.drawString(Double.toString(odo.getAng()),0,3);
                        */
                }
                
                else if (isLine(left,1)) {
                       LCD.drawString("left one         ", 0, 1);
                        do {
                        	leftMotor.stop();
                        } while (!(isLine(cs2.getNormalizedLightValue(),2)));
                        
                        leftMotor.setSpeed(50);
                        rightMotor.setSpeed(50);
                        leftMotor.forward();
                        rightMotor.forward();
                        
                }
                
                else if (isLine(right,2)) {
                        LCD.drawString("right one        !",0,1);
                        do {
                        	leftMotor.stop();
                        } while (!(isLine(cs1.getNormalizedLightValue(),1)));
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
        public boolean isLine(double light, int line){
                /* Calibration for the right sensors's values
                 *  The values for the red are always sensorCorrection units higher than the value returned by the left sensor
                 */
                if (line==2){
                        light = light - sensorCorrection;
                }
                
                /* If the values are within a threshold, return that it's a line **/
                if (light < lightThreshold ) {
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
        
        /** Updates the odometer, after going over a line 
         * 
         */
        public void updateOdo(){
        	double x = odo.getX();
        	double y = odo.getY();
        	double line;
        	double position; 
        	
        	// if the robot is going along the y-direction, update the y-direction
        	// also, update the angle
        	if ((odo.getAng()<0-angleThreshold && odo.getAng()>0+angleThreshold) || (odo.getAng()<180-angleThreshold&& odo.getAng()>180+angleThreshold)){
        		line = x % 30.48; 
        		position = line*30.48;
        		odo.setPosition(new double [] {0.0, position , odo.getAng()}, new boolean [] {false, true, false});	
        	}
        	
        	// if the robot is going along the x-direction, update the x-direction
        	// also, update the angle
        	else {
        		line = x % 30.48; 
        		position = line*30.48;
        		odo.setPosition(new double [] {position, 0.0 , odo.getAng()}, new boolean [] {true, false, false});	
        	}
        	
        }
        
        /** 
         * 
         * @return the status of the robot's light localization. Returns true if the robot is localizing and false othewise
         */
        public boolean isLocalizing(){
                return this.localizing;
        }
        
        
}
