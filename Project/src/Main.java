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
				ObjectDetection objdet = new ObjectDetection(us1, us2);
				boolean blue;
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
		} while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
