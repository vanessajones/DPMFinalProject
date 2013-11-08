import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

/** Main class to run the code 
 * 
 * @author Vanessa Jones, Christopher Petryna, Simon Lei, Taylor Dotsikas, Muhammad Hannan and Caroline Wu
 * @version  1.0 November 2 2013
 */
public class Main {
	// code for testing the object detection
	public static void main(String[] args) {
		
		do {(new Thread() {
			public void run() {
				UltrasonicSensor us1 = new UltrasonicSensor(SensorPort.S1);
				UltrasonicSensor us2 = new UltrasonicSensor(SensorPort.S2);
				Odometer odo = new Odometer(Motor.A, Motor.B, 30, true);
				ObjectDetection objdet = new ObjectDetection(us1, us2);
				Navigation navi = new Navigation(odo, objdet);
				
				while(true) {
					navi.goFindBlock();
					navi.bringToDropZone();
				}
			}
				
			}).start();
		} while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
