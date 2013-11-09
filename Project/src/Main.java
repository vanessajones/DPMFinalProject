import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.Sound;

/** Main class to run the code 
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  1.0 November 2 2013
 */
public class Main {
	
	private static NXTRegulatedMotor leftMotor = Motor.A;
	private static NXTRegulatedMotor rightMotor = Motor.B;
	
	// code for testing the object detection
	public static void main(String[] args) {
		
				
				UltrasonicSensor us1 = new UltrasonicSensor(SensorPort.S1);
				UltrasonicSensor us2 = new UltrasonicSensor(SensorPort.S2);
				ColorSensor cs1 = new ColorSensor(SensorPort.S3);
				ColorSensor cs2 = new ColorSensor(SensorPort.S4);
				//ObjectDetection objdet = new ObjectDetection(us1, us2);
				//boolean blue;
				//HandleBlock lift = new HandleBlock();
				LightLocalizer light = new LightLocalizer(cs1,cs2, leftMotor, rightMotor);
				while (true){
					light.doLocalization();
					try {Thread.sleep(50);} catch (Exception E) {}
				}
			//	lift.lift();
		
				/*
				LCD.drawChar('a', 0, 0);
				while (true) {
					blue = objdet.isBlue();
					if (blue) {
						LCD.drawString("true ", 0, 0);
					}
					else {
						LCD.drawString("false", 0, 0);
					}
					try {Thread.sleep(100);} catch (Exception E) {}
				}
				}
			}).start();
		
		} 
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
	*/
	}
}
