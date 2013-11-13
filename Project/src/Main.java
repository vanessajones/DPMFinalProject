import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.Sound;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

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
               us.doLocalization();
               navi.turnTo(90, true);
               
               while(true) {
            	   navi.goFindBlock();
            	   navi.bringToDropZone();
              
               }    
               

       }
}