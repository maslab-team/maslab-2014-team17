package tablet;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.InputStream;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

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
		Mat rawImage = this.look();

		//TODO:Process the image using Hough line algorithm
		//Mat processedImage = WallImageProcessor.process(rawImage);
	
	}
	
	/**
	 * Detect balls.
	 */
	void detectBalls() {
		Mat rawImage = this.look();

		//TODO:Process the image using Hough circle algorithm
		//Mat processedImage = WallImageProcessor.process(rawImage);
		
	}
	
	/**
	 * Capture image.
	 */
	Mat look() {
       Mat rawImage = new Mat();
        
        while (!camera.read(rawImage)) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
	        
	    return rawImage;
	}
	
	/*
	 * Canny line transform.
	 */
	Mat Canny(Mat image) {
	    Mat image_gray = new Mat();
	    Imgproc.cvtColor(image, image_gray, Imgproc.COLOR_BGR2GRAY);
	    return image_gray;
	}
	
	void view(Mat image) {
//	    Mat image_tmp = image;
//        MatOfByte matOfByte = new MatOfByte();
//
//        Highgui.imencode(".jpg", image_tmp, matOfByte); 
//
//        byte[] byteArray = matOfByte.toArray();
//        BufferedImage bufImage = null;
//
//        try {
//            InputStream in = new ByteArrayInputStream(byteArray);
//            bufImage = ImageIO.read(in);
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
//        Graphics g = (Graphics2D) bufImage.getGraphics();
//        g.drawImage(bufImage , 0, 0, null);
	    
	    Imgproc.resize(image, image, new Size(640, 480));
	    MatOfByte matOfByte = new MatOfByte();
	    Highgui.imencode(".jpg", image, matOfByte);
	    byte[] byteArray = matOfByte.toArray();
	    BufferedImage bufImage = null;
	    try {
	        InputStream in = new ByteArrayInputStream(byteArray);
	        bufImage = ImageIO.read(in);
	        JFrame frame = new JFrame();
	        frame.getContentPane().add(new JLabel(new ImageIcon(bufImage)));
	        frame.pack();
	        frame.setVisible(true);
	        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    } catch (Exception e) {
	        e.printStackTrace();
	    }
	}
	
	public static void main(String[] args) {
	    RobotEye eye = new RobotEye(0);
	    eye.view(eye.Canny(eye.look()));
	}

}
