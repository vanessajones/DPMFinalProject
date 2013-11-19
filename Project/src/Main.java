import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.Sound;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.LCD;
import lejos.nxt.comm.*;


/** Main class to run the code 
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  1.0 November 2 2013
 */
public class Main {
  
  private static NXTRegulatedMotor leftMotor = Motor.A;
  private static NXTRegulatedMotor rightMotor = Motor.B;
  private static NXTRegulatedMotor lifting = Motor.C;
  
        // code for testing the object detection
        public static void main(String[] args) {
                
               UltrasonicSensor us1 = new UltrasonicSensor(SensorPort.S1);
               UltrasonicSensor us2 = new UltrasonicSensor(SensorPort.S2);
               ColorSensor cs1 = new ColorSensor(SensorPort.S3);
               ColorSensor cs2 = new ColorSensor(SensorPort.S4);
                        
               Odometer odo = new Odometer(Motor.A, Motor.B, 30, true);
               ObjectDetection objdet = new ObjectDetection(us1, us2, odo);
               USLocalizer us = new USLocalizer(odo, us1);
              
               LightLocalizer ls = new LightLocalizer(cs1,cs2, leftMotor, rightMotor, odo);
               HandleBlock handle = new HandleBlock(lifting);
               Navigation navi = new Navigation(odo, objdet, ls, handle);
             
               BluetoothConnection conn = new BluetoothConnection();
               Transmission t = conn.getTransmission();
               if (t == null) {
                 LCD.drawString("Failed to read transmission", 0, 5);
               } else {
                  StartCorner corner = t.startingCorner;
                  PlayerRole role = t.role;
                  // green zone is defined by these (bottom-left and top-right) corners:
                  int[] greenZone = t.greenZone;
   
                  // red zone is defined by these (bottom-left and top-right) corners:
                  int[] redZone = t.redZone;
   
                  // print out the transmission information to the LCD
                  conn.printTransmission();
                 
                 
                 /** Ultrasonic Sensor Localization **/
                 /** Update the odometry, relative to the starting corner **/
                 us.doLocalization(t.getId());
                 LCD.refresh;
                 LCD.drawString(Double.toString(odo.getX()),0,1);
                 LCD.drawString(Double.toString(odo.getY(),0,2);
                 LCD.drawString(Double.toString(odo.getAng(),0,3);
                 /** Light Sensor localization **/
                 navi.turnTo(0,false);
                 ls.doLocalization();
                 navi.turnTo(90,false);
                 ls.doLocalization();
            
                 /** Go Find Blocks **/
                while(true) {
                navi.goFindBlock();
                navi.bringToDropZone();
  

       }
}
