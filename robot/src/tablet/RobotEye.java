package tablet;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;

/**
 * Contains methods for detecting objects in the game arena,
 * using images from a camera.
 * 
 * The RobotEye can be initialized with a parameter that specifies
 * the camera.  On Linux machines, the RobotEye will use the camera
 * connected at /dev/videoN where N is the parameter that specifies
 * the camera.
 * 
 * For the Samsung Slate Series 7, USB cameras, when plugged in once
 * after boot up, are specified by the parameter 2.  The back-facing
 * tablet camera is 0, and the front-facing camera is 1.
 * 
 * detectWalls and detectBalls should be called (ideally continuously)
 * from a Thread which updates the presumed positions of walls and balls,
 * used by the main robot program to make decisions.
 * 
 * @author vmayar
 *
 */
public class RobotEye {
	
	VideoCapture camera;
	
	/**
	 * Given the camera number, sets up the camera for this
	 * instance of RobotEye.  Note that the USB camera
	 * is usually camera 2, but may be greater than 2.
	 * 
	 * @param n	The camera number.
	 */
	RobotEye(int n) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		// Setup the camera
		camera = new VideoCapture();
		camera.open(n);
	}
	
	/**
	 * Constructor which by default initializes the camera
	 * settings using the front-facing camera (camera 1).
	 */
	RobotEye() {
		this(1);
	}
	
	/**
	 * Detects walls.
	 */
	void detectWalls() {
		Mat rawImage = new Mat();
		
		while (!camera.read(rawImage)) {
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		//TODO:Process the image using Hough line algorithm
		//Mat processedImage = WallImageProcessor.process(rawImage);
	
	}
	
	/**
	 * Detect balls.
	 */
	void detectBalls() {
		Mat rawImage = new Mat();
		
		while (!camera.read(rawImage)) {
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		//TODO:Process the image using Hough circle algorithm
		//Mat processedImage = WallImageProcessor.process(rawImage);
		
	}

}
