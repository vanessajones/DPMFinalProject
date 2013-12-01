
import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.LCD;

/** Main class to run the robot's program
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  3.0 November 28 2013
 */
public class Main {
  
 /* Initialize the robot's motors */
  private static NXTRegulatedMotor leftMotor = Motor.A;
  private static NXTRegulatedMotor rightMotor = Motor.B;
  private static NXTRegulatedMotor lifting = Motor.C;
  
        public static void main(String[] args) {
                
        		/* Initialize the light and ultrasonic sensors */
               UltrasonicSensor us1 = new UltrasonicSensor(SensorPort.S1);
               UltrasonicSensor us2 = new UltrasonicSensor(SensorPort.S2);
               ColorSensor cs1 = new ColorSensor(SensorPort.S3);
               ColorSensor cs2 = new ColorSensor(SensorPort.S4);
                 
               /* Initialize the odometer, object detection, light localizer and block handeling */
               Odometer odo = new Odometer(Motor.A, Motor.B, 30, true);
               ObjectDetection objdet = new ObjectDetection(us1, us2, odo);
               LightLocalizer ls = new LightLocalizer(cs1,cs2, leftMotor, rightMotor, odo);
               HandleBlock handle = new HandleBlock(lifting);

               /* Start bluetooth connection */
               BluetoothConnection conn = new BluetoothConnection();
               Transmission t = conn.getTransmission();
               
               /* If the robot cannot read the transmission, print error message */
               if (t == null) {
                 LCD.drawString("Failed to read transmission", 0, 5);
               } 
               
               /* Acquire the bluetooth information */
               else {
            	   
                  StartCorner corner = t.startingCorner;
                  PlayerRole role = t.role;
                  // green zone is defined by these (bottom-left and top-right) corners:
                  int[] greenZone = t.greenZone;
   
                  // red zone is defined by these (bottom-left and top-right) corners:
                  int[] redZone = t.redZone;
   
                  // print out the transmission information to the LCD
                  // conn.printTransmission();
              
                  Navigation navi = new Navigation(odo, objdet, ls, handle, t.role.getId(), greenZone, redZone);
                  USLocalizer us = new USLocalizer(odo, us1, navi, ls);
                  
                  /* Perform the initial localization
                   * First, perform ultrasonic localization and update the robot's position according to the starting corner
                   * Then, perform light localization to correct the odometer and to start the robot exactly on (0,0)
                   */
                 us.doLocalization(t.startingCorner.getId());

                 /* Make the robot find blocks and bring them to the drop zone, indefinitely */
                while(true) {
                	navi.goFindBlock();
                	navi.bringToDropZone();
                }
              }
        }
     }
