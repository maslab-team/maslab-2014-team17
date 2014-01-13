package tablet;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
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
	
	/** Hough line constants. */
	private static final double HOUGH_RHO = 1;
	private static final double HOUGH_THETA = Math.PI/180;
	private static final double HOUGH_MIN_LINE_LENGTH = 35;
	private static final double HOUGH_MAX_LINE_GAP = 5;
	private static final int HOUGH_LINE_DETECTION_THRESH = 30;

	/** Hough circle constants. */
	private static final double HOUGH_INVERSE_ACCUMULATOR_RES = 1;
	private static final double HOUGH_MIN_CENTER_DIST = 25;
	private static final double HOUGH_CIRCLE_DETECTION_THRESH = 30;
	private static final int HOUGH_MIN_RADIUS = 15;
	private static final int HOUGH_MAX_RADIUS = 100;
	private static final int HOUGH_MAX_NUM_CIRCLES = 6;
	
	/** Canny constants. */
	private static final double CANNY_LOW_THRESH = 80.0;
	private static final double CANNY_HIGH_THRESH = 160.0;
	
	/** Gaussian blur constants. */
	private static final int BLUR_KERNEL_SIZE = 5;
	
	/** Display constants. */
	private static final int DISPLAY_THICKNESS = 3;
	
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
	 * Detects walls, given an image processed through the Canny algorithm.
	 * 
	 * Returns lines as an array of arrays of integers.  Each entry in the
	 * returned array represents a line as an array [x0,y0,x1,y1], where
	 * (x0,y0) and (x1,y1) are the endpoints of the line.
	 * 
	 */
	int[][] detectWalls(Mat image) {
		final int DATA_LENGTH = 4;
		Mat lines = new Mat();
		int numCols;
		int[][] ret;
		
		Imgproc.HoughLinesP(image, lines, HOUGH_RHO, HOUGH_THETA, HOUGH_LINE_DETECTION_THRESH,
				HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);
		
		numCols = lines.cols();
		ret = new int[numCols][DATA_LENGTH];
		System.out.printf("Identified %d lines.\n", numCols);
		
		for(int lineIdx = 0; lineIdx < numCols; ++lineIdx) {
			lines.get(0, lineIdx, ret[lineIdx]);
		}
		
		return ret;
	}
	
	/**
	 * Detect balls, given a blurred gray-scale image.
	 * 
	 * Returns circles as an array of arrays of floats.  Each entry in the
	 * returned array represents a circle as an array [x0,y0,r], where
	 * (x0,y0) is the center of the circle and r is the radius.
	 */
	float[][] detectBalls(Mat image) {
		final int DATA_LENGTH = 3;
		Mat circles = new Mat();
		int numCols;
		float[][] ret;
		
		Imgproc.HoughCircles(image, circles, Imgproc.CV_HOUGH_GRADIENT,
				HOUGH_INVERSE_ACCUMULATOR_RES, HOUGH_MIN_CENTER_DIST);/*, CANNY_HIGH_THRESH,
				HOUGH_CIRCLE_DETECTION_THRESH, HOUGH_MIN_RADIUS, HOUGH_MAX_RADIUS);*/
		
		numCols = circles.cols();
		System.out.printf("Identified %d circles.\n", numCols);
		// Use only the *best* guesses for circles, which are first in the returned Mat.
		if(numCols > HOUGH_MAX_NUM_CIRCLES) numCols = HOUGH_MAX_NUM_CIRCLES;
		ret = new float[numCols][DATA_LENGTH];
		
		for(int circleIdx = 0; circleIdx < numCols; ++circleIdx) {
			circles.get(0, circleIdx, ret[circleIdx]);
		}
		
		return ret;	
	}
	
	/**
	 * Runs detectWalls on cannyImage, which has been run through the
	 * Canny algorithm, and uses the canvasImage to produce an image
	 * with walls outlined.
	 * 
	 * @param cannyImage
	 * @param canvasImage
	 * @return
	 */
	Mat detectWallsForDisplay(Mat cannyImage, Mat canvasImage) {
		Mat processedImage = new Mat();
		canvasImage.copyTo(processedImage);
		int[][] wallData = detectWalls(cannyImage);
		int numLines = wallData.length;
		
		for(int lineIdx = 0; lineIdx < numLines; ++lineIdx) {
			int[] lineData = wallData[lineIdx];
			Core.line(processedImage, new Point(lineData[0], lineData[1]),
				new Point(lineData[2], lineData[3]), new Scalar(0, 0, 255),
				DISPLAY_THICKNESS);
		}
		
		return processedImage;
	}
	
	/**
	 * Runs detectBalls on grayImage, a blurred, gray-scale image,
	 * and uses the canvasImage to produce an image with balls outlined.
	 * 
	 * @param blurredGrayImage
	 * @param canvasImage
	 * @return
	 */
	Mat detectBallsForDisplay(Mat blurredGrayImage, Mat canvasImage) {
		Mat processedImage = new Mat();
		canvasImage.copyTo(processedImage);
		float[][] ballData = detectBalls(blurredGrayImage);
		int numCircles = ballData.length;
		
		for(int circleIdx = 0; circleIdx < numCircles; ++circleIdx) {
			float[] circleData = ballData[circleIdx];
			Core.circle(processedImage, new Point(circleData[0], circleData[1]),
					(int)circleData[2], new Scalar(0, 255, 0), DISPLAY_THICKNESS);
			System.out.printf("Circle:\tcenter: (%.1f,%.1f),\tradius: %.1f\n", circleData[0], circleData[1], circleData[2]);
		}
		
		return processedImage;
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
	
	/**
	 * Canny transform.
	 * 
	 * @param image The image to transform.
	 * @return The transformed image.
	 */
	Mat Canny(Mat image) {
	    Mat cannyImage = new Mat();
	    Imgproc.Canny(image, cannyImage, CANNY_LOW_THRESH, CANNY_HIGH_THRESH);
	    return cannyImage;
	}
	
	/**
	 * Transform an image to grayscale.
	 * 
	 * @param image The image to transform.
	 * @return The transformed image.
	 */
	Mat gray(Mat image) {
	    Mat grayImage = new Mat();
	    Imgproc.cvtColor(image, grayImage, Imgproc.COLOR_BGR2GRAY);
	    return grayImage;
	}
	
	Mat blur(Mat image) {
		Mat blurryImage = new Mat();
		Imgproc.medianBlur(image, blurryImage, BLUR_KERNEL_SIZE);
		return blurryImage;
	}
	
	/**
	 * Displays an image in a GUI.
	 * 
	 * @param image The image to display.
	 */
	void view(Mat image) {
		InputStream in = null;
		JFrame frame = null;
	    Imgproc.resize(image, image, new Size(640, 480));
	    MatOfByte matOfByte = new MatOfByte();
	    Highgui.imencode(".jpg", image, matOfByte);
	    byte[] byteArray = matOfByte.toArray();
	    BufferedImage bufImage = null;
	    try {
	        in = new ByteArrayInputStream(byteArray);
	        bufImage = ImageIO.read(in);
	        frame = new JFrame();
	        frame.getContentPane().add(new JLabel(new ImageIcon(bufImage)));
	        frame.pack();
	        frame.setVisible(true);
	        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    } catch (IOException e) {
	        e.printStackTrace();
	    } finally {
	    	if(in != null) {
	    		try {
	    			in.close();
	    		} catch(IOException e) {
	    			e.printStackTrace();
	    		}
	    	}
	    }
	}
	
	public static void main(String[] args) {
	    RobotEye eye = new RobotEye(0);
	    Mat canvasImage = new Mat();
	    Mat sourceImage = eye.look();
	    Mat grayImage = eye.gray(sourceImage);
	    Mat blurredGrayImage = eye.blur(grayImage);
	    Mat cannyImage = eye.Canny(sourceImage);
	    cannyImage.copyTo(canvasImage);
	    Imgproc.cvtColor(canvasImage, canvasImage, Imgproc.COLOR_GRAY2BGR);
	    Mat tmpImage = eye.detectWallsForDisplay(cannyImage, canvasImage);
	    Mat processedImage = eye.detectBallsForDisplay(blurredGrayImage, tmpImage);
	    
	    eye.view(blurredGrayImage);
	    eye.view(processedImage);
	}

}
