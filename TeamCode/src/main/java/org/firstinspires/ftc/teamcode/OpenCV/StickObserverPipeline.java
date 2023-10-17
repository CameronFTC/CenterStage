package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//for dashboard
@Config
public class StickObserverPipeline extends OpenCvPipeline {

    private static Point contourCoords;
    private String color;
    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;
    public static double xCoord = 0;
    public static double yCoord = 0;

    public StickObserverPipeline(String color) {
        frameList = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
        Scalar lowHSV; // lenient lower bound HSV for yellow 20,70,80
        Scalar highHSV; // lenient higher bound HSV for yellow 32,255,255
        //blue low: 100, 150, 0
        //blue high: 140, 255, 255

        Mat thresh = new Mat();
        //remove for blue

        if(color.equals("Red"))
        {
            lowHSV = new Scalar(0, 70, 50);
            highHSV = new Scalar(30, 255, 255);
            Core.inRange(mat, lowHSV, highHSV, thresh);

            Scalar lowHSV2 = new Scalar(150, 70, 50);
            Scalar highHSV2 = new Scalar(180, 255, 255);
            Core.inRange(mat, lowHSV2, highHSV2, thresh);
        }
        else if(color.equals("Blue"))
        {
            lowHSV = new Scalar(100, 150, 0);
            highHSV = new Scalar(140, 255, 255);
            Core.inRange(mat, lowHSV, highHSV, thresh);
        }
        else if(color.equals("White"))
        {
            lowHSV = new Scalar(0, 0, 168);
            highHSV = new Scalar(172, 111, 255);
            Core.inRange(mat, lowHSV, highHSV, thresh);
        }

        // Get a black and white image of yellow objects

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }


        //release all the data
        input.release();
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        edges.release();
        thresh.release();
        finalMask.release();
        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
        double maxContour = 0;
        MatOfPoint largestContour = new MatOfPoint();
        for(MatOfPoint contour : contours)
        {
            if(Imgproc.contourArea(contour) > maxContour)
            {
                maxContour = Imgproc.contourArea(contour);
                largestContour = contour;
            }
        }

        Rect rect = Imgproc.boundingRect(largestContour);
        Imgproc.rectangle(input, rect, new Scalar(255,0, 0));
        xCoord = rect.x + rect.width / 2;
        yCoord = rect.y + rect.height / 2;

        //List<Point> coords = new ArrayList<>();
        //Converters.Mat_to_vector_Point(largestContour, coords);
        //contourCoords = coords.get(0);

        return input;
    }

    public static Point getContourCoords()
    {
        return contourCoords;
    }

}