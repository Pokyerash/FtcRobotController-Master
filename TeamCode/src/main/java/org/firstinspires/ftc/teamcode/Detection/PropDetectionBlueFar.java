package org.firstinspires.ftc.teamcode.Detection;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class PropDetectionBlueFar implements VisionProcessor {

    public int detection = 1;
    //centru
    public static int leftRectX1 = 200, leftRectY1 = 325;         // 250, 100
    public static int leftRectX2 = 350, leftRectY2 = 500;           // 450, 300

    public static double rightThresh = 400000;     //500000
    public double rightSum = 0;
    //dreapta
    public static int middleRectX1 = 770, middleRectY1 =340;         // 650, 200
    public static int middleRectX2 = 920, middleRectY2 = 550;        // 850, 400


    public static double middleThresh =500000;       //250
    public double middleSum = 0;

    public static int blueLowH = 110, blueLowS = 160, blueLowV = 0;
    public static int blueHighH = 125, blueHighS = 255, blueHighV = 255;

    Mat workingMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, workingMat, Imgproc.COLOR_RGB2HSV);

        Rect rightRect = new Rect(new Point(leftRectX1, leftRectY1), new Point(leftRectX2, leftRectY2));
        Rect middleRect = new Rect(new Point(middleRectX1, middleRectY1), new Point(middleRectX2, middleRectY2));

        Scalar lowThresh = new Scalar(blueLowH, blueLowS, blueLowV);
        Scalar highThresh = new Scalar(blueHighH, blueHighS, blueHighV);

        Core.inRange(workingMat, lowThresh, highThresh, workingMat);

        rightSum = Core.sumElems(workingMat.submat(rightRect)).val[0];
        middleSum = Core.sumElems(workingMat.submat(middleRect)).val[0];

        Imgproc.rectangle(frame, rightRect, new Scalar(0,255,0), 5);
        Imgproc.rectangle(frame, middleRect, new Scalar(0,255,0), 5);

     /*   if(rightSum >rightThresh && rightSum>900000 && middleSum<middleThresh)
            detection = 1;
        else if (middleSum > middleThresh && rightSum<900000)
            detection = 2;
        else //if (middleSum > middleThresh && rightSum<6500000)
            detection = 3; */

        if(rightSum > rightThresh)
            detection = 2;
        else if (middleSum > middleThresh)
            detection = 1;
        else detection = 3;

//        workingMat.copyTo(frame);

        workingMat.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
/*
package org.firstinspires.ftc.teamcode.Detection;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class PropDetectionBlueFar implements VisionProcessor {

    public int detection = 1;

    public static int rightRectX1 = 450, rightRectY1 = 350;
    public static int rightRectX2 = 550, rightRectY2 = 450;

    public static double rightThresh = 400000;
    public double rightSum = 0;

    public static int middleRectX1 = 820, middleRectY1 = 420;
    public static int middleRectX2 = 920, middleRectY2 = 520;

    public static double middleThresh = 500000;
    public double middleSum = 0;

    public static int blueLowH = 110, blueLowS = 160, blueLowV = 0;
    public static int blueHighH = 125, blueHighS = 255, blueHighV = 255;

    Mat workingMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, workingMat, Imgproc.COLOR_BGR2HSV);

        Rect rightRect = new Rect(new Point(rightRectX1, rightRectY1), new Point(rightRectX2, rightRectY2));
        Rect middleRect = new Rect(new Point(middleRectX1, middleRectY1), new Point(middleRectX2, middleRectY2));

        Scalar lowThresh = new Scalar(blueLowH, blueLowS, blueLowV);
        Scalar highThresh = new Scalar(blueHighH, blueHighS, blueHighV);

        Core.inRange(workingMat, lowThresh, highThresh, workingMat);

        rightSum = Core.sumElems(workingMat.submat(rightRect)).val[0];
        middleSum = Core.sumElems(workingMat.submat(middleRect)).val[0];

        Imgproc.rectangle(frame, rightRect, new Scalar(0,255,0), 5);
        Imgproc.rectangle(frame, middleRect, new Scalar(0,255,0), 5);

        if(rightSum > rightThresh)
            detection = 3;
        else if (middleSum > middleThresh)
            detection = 2;
        else detection = 1;

//        workingMat.copyTo(frame);

        workingMat.release();

        return detection;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

           }
}*/
