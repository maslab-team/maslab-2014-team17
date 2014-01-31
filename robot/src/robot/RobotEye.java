package robot;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
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

	/** Turn on to enable display. */
	private static final boolean DISPLAY = true;
	
	/** Hough line constants. */
	private static final double HOUGH_RHO = 1;
	private static final double HOUGH_THETA = Math.PI/180.0;
	private static final double HOUGH_MIN_LINE_LENGTH = 35;
	private static final double HOUGH_MAX_LINE_GAP = 5;
	private static final int HOUGH_LINE_DETECTION_THRESH = 30;

	/** Hough circle constants. */
	private static final double HOUGH_INVERSE_ACCUMULATOR_RES = 1;
	private static final double HOUGH_MIN_CENTER_DIST = 80;
	private static final double HOUGH_CIRCLE_DETECTION_THRESH = 15;
	private static final int HOUGH_MIN_RADIUS = 10;
	private static final int HOUGH_MAX_RADIUS = 150;
	private static final int HOUGH_MAX_NUM_CIRCLES = 6;
	
	/** Canny constants. */
	private static final double CANNY_LOW_THRESH = 150.0;
	private static final double CANNY_HIGH_THRESH = 300.0;
	
	/** Gaussian blur constants. */
	private static final int BLUR_KERNEL_SIZE = 13;
	
	/** Display constants. */
	private static final int DISPLAY_THICKNESS = 3;
	
	/** Color constants. */
	private static final int RED_BALL_HUE_LOW = 10;
	private static final int RED_BALL_HUE_HIGH = 245;
	private static final int GREEN_BALL_HUE_LOW = 45;
	private static final int GREEN_BALL_HUE_HIGH = 85;
	private static final int GREEN_BALL_SATURATION_LOW = 64;
	private static final int GREEN_BALL_SATURATION_HIGH = 192;
	private static final int GREEN_BALL_VALUE_LOW = 20;
	private static final int GREEN_BALL_VALUE_HIGH = 192;
	private static final int RED_BALL_SATURATION_LOW = 64;
	private static final int RED_BALL_SATURATION_HIGH = 255;
	private static final int RED_BALL_VALUE_LOW = 20;
	private static final int RED_BALL_VALUE_HIGH = 245;
	
	/** Image source constants. */
	static final int IMAGE_HEIGHT = 1080; // 1080 for webcam, 720 for macbook
	static final int IMAGE_WIDTH = 1920; // 1920 for webcam, 1280 for macbook
	static final double IMAGE_HORIZONTAL_ANGLE_OF_VIEW = 120.0 * Math.PI / 180.0;
	static final double IMAGE_DEPTH_IN_PIXELS =
			IMAGE_WIDTH/(2.0*Math.tan(IMAGE_HORIZONTAL_ANGLE_OF_VIEW/2.0));
	
	VideoCapture camera;
	Map<String, JFrame> frames; // Frames used for display.
	
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

		if(DISPLAY) {
			setUpDisplay();
		}
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
	 * Returns lines as an list of lists of integers.  Each entry in the
	 * returned list represents a line as an list [x0,y0,x1,y1], where
	 * (x0,y0) and (x1,y1) are the endpoints of the line.
	 * 
	 */
	List<List<Integer>> detectWalls(Mat image) {
		final int DATA_LENGTH = 4;
		Mat lines = new Mat();
		int numCols;
		List<List<Integer>> ret = new ArrayList<List<Integer>>();
		
		Imgproc.HoughLinesP(image, lines, HOUGH_RHO, HOUGH_THETA, HOUGH_LINE_DETECTION_THRESH,
				HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);
		
		numCols = lines.cols();
		int[] data = new int[DATA_LENGTH];
		//System.out.printf("Identified %d lines.\n", numCols);
		
		for(int lineIdx = 0; lineIdx < numCols; ++lineIdx) {
			lines.get(0, lineIdx, data);
			List<Integer> dataList = new ArrayList<Integer>();
			for(int el : data) {
				dataList.add(el);
			}
			ret.add(dataList);
		}
		
		return ret;
	}
	
	/**
	 * Detect balls, given a blurred gray-scale image.
	 * 
	 * Returns circles as a list of lists of floats.  Each entry in the
	 * returned list represents a circle as an list [x0,y0,r], where
	 * (x0,y0) is the center of the circle and r is the radius.
	 */
	List<List<Float>> detectBalls(Mat image) {
		final int DATA_LENGTH = 3;
		Mat circles = new Mat();
		int numCols;
		List<List<Float>> ret = new ArrayList<List<Float>>();
		
		Imgproc.HoughCircles(image, circles, Imgproc.CV_HOUGH_GRADIENT,
				HOUGH_INVERSE_ACCUMULATOR_RES, HOUGH_MIN_CENTER_DIST, CANNY_HIGH_THRESH,
				HOUGH_CIRCLE_DETECTION_THRESH, HOUGH_MIN_RADIUS, HOUGH_MAX_RADIUS);
		
		numCols = circles.cols();
		//System.out.printf("Identified %d circles.\n", numCols);
		// Use only the *best* guesses for circles, which are first in the returned Mat.
		if(numCols > HOUGH_MAX_NUM_CIRCLES) numCols = HOUGH_MAX_NUM_CIRCLES;
		float[] data = new float[DATA_LENGTH];
		
		for(int circleIdx = 0; circleIdx < numCols; ++circleIdx) {
			circles.get(0, circleIdx, data);
			List<Float> dataList = new ArrayList<Float>();
			for(float el : data) {
				dataList.add(el);
			}
			ret.add(dataList);
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
		List<List<Integer>> wallData = detectWalls(cannyImage);
		int numLines = wallData.size();
		
		for(int lineIdx = 0; lineIdx < numLines; ++lineIdx) {
			List<Integer> lineData = wallData.get(lineIdx);
			Core.line(processedImage, new Point(lineData.get(0), lineData.get(1)),
				new Point(lineData.get(2), lineData.get(3)), new Scalar(0, 0, 255),
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
	Mat detectBallsForDisplay(Mat blurredGrayImage, Mat canvasImage, Scalar color) {
		Mat processedImage = new Mat();
		canvasImage.copyTo(processedImage);
		List<List<Float>> ballData = detectBalls(blurredGrayImage);
		int numCircles = ballData.size();
		
		for(int circleIdx = 0; circleIdx < numCircles; ++circleIdx) {
			List<Float> circleData = ballData.get(circleIdx);
			Core.circle(processedImage, new Point(circleData.get(0), circleData.get(1)),
					(int)((float)circleData.get(2)), color, DISPLAY_THICKNESS);
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
	
	/**
	 * Blur an image with a medium blur.
	 * 
	 * @param image The image to transform.
	 * @return The transformed image.
	 */
	Mat blur(Mat image) {
		Mat blurryImage = new Mat();
		Imgproc.medianBlur(image, blurryImage, BLUR_KERNEL_SIZE);
		return blurryImage;
	}
	
	/**
	 * Return a binary image of the pixels in the specified range.
	 * 
	 * @param image
	 * @return
	 */
	Mat inRange(Mat image, Scalar lower, Scalar upper) {
		Mat processedImage = new Mat();
		Core.inRange(image, lower, upper, processedImage);
		return processedImage;
	}

	private void setUpDisplay() {
		frames = new HashMap<String, JFrame>();
		frames.put("processed", getDefaultFrame(0, 0));
		frames.put("red", getDefaultFrame(480, 0));
		frames.put("green", getDefaultFrame(0, 360));
	}
	
	private JFrame getDefaultFrame(int xLoc, int yLoc) {
		JFrame frame = new JFrame();
		frame.setLocation(xLoc, yLoc);
        frame.setVisible(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        return frame;
	}
	
	/**
	 * Displays an image in a GUI.
	 * 
	 * @param image The image to display.
	 */
	void view(Mat image, String frameString) {
		JFrame frame = frames.get(frameString);
		if(frame == null) {	return;	}
		
		ByteArrayInputStream in = null;
	    Imgproc.resize(image, image, new Size(480, 360));
	    MatOfByte matOfByte = new MatOfByte();
	    Highgui.imencode(".jpg", image, matOfByte);
	    byte[] byteArray = matOfByte.toArray();
	    BufferedImage bufImage = null;
	    try {
	        in = new ByteArrayInputStream(byteArray);
	        bufImage = ImageIO.read(in);
	        frame.getContentPane().removeAll();
	        frame.getContentPane().add(new JLabel(new ImageIcon(bufImage)));
	        frame.pack();
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
	
	/**
	 * Processes the next available image and returns
	 * an instance of RobotEye.Data with the detected lines,
	 * red circles, and green circles.
	 * 
	 */
	Data process() {
		Data data = new Data();
	    Mat tmpImage = new Mat();
	    Mat redTmpImage = new Mat();
	    Mat redTmpImageLow = new Mat();
	    Mat redTmpImageHigh = new Mat();
	    Mat greenTmpImage = new Mat();
	    Mat sourceImage = this.blur(this.look());
	    Imgproc.cvtColor(sourceImage, tmpImage, Imgproc.COLOR_BGR2HSV_FULL);
	    redTmpImageLow = this.inRange(tmpImage, new Scalar(0,
	    		RED_BALL_SATURATION_LOW, RED_BALL_VALUE_LOW),
	    		new Scalar(RED_BALL_HUE_LOW,
	    		RED_BALL_SATURATION_HIGH, RED_BALL_VALUE_HIGH));
	    redTmpImageHigh = this.inRange(tmpImage, new Scalar(RED_BALL_HUE_HIGH,
                RED_BALL_SATURATION_LOW, RED_BALL_VALUE_LOW),
                new Scalar(255,
                RED_BALL_SATURATION_HIGH, RED_BALL_VALUE_HIGH));
	    Core.bitwise_or(redTmpImageLow, redTmpImageHigh, redTmpImage);
	    Mat redBlurredGrayImage = this.blur(redTmpImage);
	    greenTmpImage = this.inRange(tmpImage, new Scalar(GREEN_BALL_HUE_LOW,
	    		GREEN_BALL_SATURATION_LOW, GREEN_BALL_VALUE_LOW),
	    		new Scalar(GREEN_BALL_HUE_HIGH,
	    		GREEN_BALL_SATURATION_HIGH, GREEN_BALL_VALUE_HIGH));
	    Mat greenBlurredGrayImage = this.blur(greenTmpImage);
	    data.addRedCircles(this.detectBalls(redTmpImage));
	    data.addGreenCircles(this.detectBalls(greenBlurredGrayImage));
	    
	    if(DISPLAY) {
	    	Mat canvasImage = new Mat();
	    	Mat displayTmpImage = new Mat();
	    	Mat processedImage = new Mat();
	    	sourceImage.copyTo(canvasImage);
	    	displayTmpImage = this.detectBallsForDisplay(redBlurredGrayImage, canvasImage,
	    			new Scalar(0, 0, 255));
	    	processedImage = this.detectBallsForDisplay(greenBlurredGrayImage, displayTmpImage,
	    			new Scalar(0, 255, 0));
	    	view(processedImage, "processed");
	    	view(redBlurredGrayImage, "red");
	    	view(greenBlurredGrayImage, "green");
	    }
	    
	    return data;
	}
	
	public static class Data {
		//List<List<Integer>> lines;
		List<List<Float>> redCircles;
		List<List<Float>> greenCircles;
		
		Data() {
			//lines = new ArrayList<List<Integer>>();
			redCircles = new ArrayList<List<Float>>();
			greenCircles = new ArrayList<List<Float>>();
		}
		
		/*void addLines(List<List<Integer>> detectedLines) {
			if(detectedLines != null) this.lines.addAll(detectedLines);
		}*/
		void addRedCircles(List<List<Float>> detectedRedCircles) {
			if(detectedRedCircles != null) this.redCircles.addAll(detectedRedCircles);
		}
		void addGreenCircles(List<List<Float>> detectedGreenCircles) {
			if(detectedGreenCircles != null) this.greenCircles.addAll(detectedGreenCircles);
		}
		
		/*public List<List<Integer>> getLines() {
			return new ArrayList<List<Integer>>(lines);
		}*/
		public List<List<Float>> getRedCircles() {
			return new ArrayList<List<Float>>(redCircles);
		}
		public List<List<Float>> getGreenCircles() {
			return new ArrayList<List<Float>>(greenCircles);
		}
	}
	
	public static void main(String[] args) {
	    
	    RobotEye eye = new RobotEye(0);
	    Mat im = eye.look();
	    eye.view(im, "cam");
	    
//	    RobotEye eye = new RobotEye(0);
//	    Mat processedImage = new Mat();
//	    Mat canvasImage = new Mat();
//	    Mat tmpImage = new Mat();
//	    Mat sourceImage = eye.look();
//	    Imgproc.cvtColor(sourceImage, tmpImage, Imgproc.COLOR_BGR2HSV_FULL);
//	    int rotation = 128 - 255;
//	    Core.add(tmpImage,  new Scalar(rotation), tmpImage);
//	    tmpImage = eye.inRange(tmpImage, new Scalar(RED_BALL_HUE-RED_BALL_HUE_TOLERANCE, 0, 0),
//	    		new Scalar(RED_BALL_HUE+RED_BALL_HUE_TOLERANCE, 255, 250));
//	    Mat blurredGrayImage = eye.blur(tmpImage);
//	    Mat cannyImage = eye.Canny(sourceImage);
//	    cannyImage.copyTo(canvasImage);
//	    Imgproc.cvtColor(canvasImage, canvasImage, Imgproc.COLOR_GRAY2BGR);
//	    Mat canvasImageWalls = eye.detectWallsForDisplay(cannyImage, canvasImage);
	    
	    
	    /*
	    FeatureDetector blob = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
	    blob.read("/Users/vmayar/Dropbox/Vinay/MASLAB/maslab-2014-team17/robot/config/blob.yml");
	    MatOfKeyPoint keypoints = new MatOfKeyPoint();
	    blob.detect(tmpImage, keypoints);
	    System.out.printf("rows: %d\t cols:%d\n", keypoints.rows(), keypoints.cols());
	    float[] data = new float[7];
	    for(int i = 0; i < keypoints.rows(); i++) {
	    	keypoints.get(i,0,data);
	    	System.out.printf("row: %d\n\t", i);
	    	for(int j =0 ; j < 7; ++j) {
	    		System.out.printf("%.2f ", data[j]);
	    	}
	    	System.out.println();
	    }
	    Features2d.drawKeypoints(sourceImage, keypoints, processedImage, new Scalar(0,255,0),0);
		
	    
	    eye.view(sourceImage);
	    eye.view(tmpImage);
	    eye.view(blurredGrayImage);
	    eye.view(processedImage);
	    */
	}

}
