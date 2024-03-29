package org.firstinspires;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * monitor: 640 x 480
 * YES
 */
@Autonomous(name= "OpenCVSkystoneDetector", group="Sky autonomous")
//comment out this line before using
public class SkystoneDetectorPhoneCam extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    public static int cubeLocation = 0;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {
        String colorSide = "red".toLowerCase();
        //Dimensions of Camera Pixels
        int rows = 640;
        int cols = 480;

        //Time Elapsed
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();


        OpenCvCamera phoneCam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();

        while (opModeIsActive()) {
            if (colorSide.equals("blue")) {
                telemetry.addData("Values", intToColor(valLeft) + "   " + intToColor(valMid) + "   " + intToColor(valRight));
                telemetry.addData("Height", rows);
                telemetry.addData("Width", cols);

                //Block is in left position, so closest to center
                if (valLeft == 0 && valMid == 255 && valRight == 255) {
                    telemetry.addData("Position" , 1);
                }

                //Block is in Middle Position
                else if (valLeft == 255 && valMid == 0 && valRight == 255) {
                    telemetry.addData("Position" , 2);
                }

                //Block is in Right Position, so furthest from center
                else if (valLeft == 255 && valMid == 255 && valRight == 0) {
                    telemetry.addData("Position" , 3);
                }
                else telemetry.addData("Position", -1);
                telemetry.update();
            }
            else if (colorSide.equals("red")) {
                telemetry.addData("Values", intToColor(valRight) + "   " + intToColor(valMid) + "   " + intToColor(valLeft));
                telemetry.addData("Height", rows);
                telemetry.addData("Width", cols);

                //Block is in left, so furthest from center
                if (valLeft == 0 && valMid == 255 && valRight == 255) {
                    telemetry.addData("Position" , 3);
                }

                //Block is in Middle Position
                else if (valLeft == 255 && valMid == 0 && valRight == 255) {
                    telemetry.addData("Position", 2);
                }

                //Block is in Right Position, so closest to Center
                else if (valLeft == 255 && valMid == 255 && valRight == 0) {
                    telemetry.addData("Position" , 1);
                }
            }
            else telemetry.addData("Position", -1);
            telemetry.update();
        }
        //Couldn't Find it / Ran out of time
    }

    public static int position (LinearOpMode opMode, String colorSide) {
        colorSide = colorSide.toLowerCase();
        //Dimensions of Camera Pixels
        int rows = 640;
        int cols = 480;

        //Time Elapsed
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();


        OpenCvCamera phoneCam;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        opMode.waitForStart();

        while (opMode.opModeIsActive() && runtime.milliseconds() <= 10000) {
            if (colorSide.equals("blue")) {
                opMode.telemetry.addData("Values", intToColor(valLeft) + "   " + intToColor(valMid) + "   " + intToColor(valRight));
                opMode.telemetry.addData("Height", rows);
                opMode.telemetry.addData("Width", cols);

                opMode.telemetry.update();

                //Block is in left position, so closest to center
                if (valLeft == 0 && valMid == 255 && valRight == 255) {
                    return 1;
                }

                //Block is in Middle Position
                else if (valLeft == 255 && valMid == 0 && valRight == 255) {
                    return 2;
                }

                //Block is in Right Position, so furthest from center
                else if (valLeft == 255 && valMid == 255 && valRight == 0) {
                    return 3;
                }
            }
            else if (colorSide.equals("red")) {
                opMode.telemetry.addData("Values", intToColor(valRight) + "   " + intToColor(valMid) + "   " + intToColor(valLeft));
                opMode.telemetry.addData("Height", rows);
                opMode.telemetry.addData("Width", cols);

                opMode.telemetry.update();

                //Block is in left, so furthest from center
                if (valLeft == 0 && valMid == 255 && valRight == 255) {
                    return 3;
                }

                //Block is in Middle Position
                else if (valLeft == 255 && valMid == 0 && valRight == 255) {
                    return 2;
                }

                //Block is in Right Position, so closest to Center
                else if (valLeft == 255 && valMid == 255 && valRight == 0) {
                    return 1;
                }
            }
        }
        //Couldn't Find it / Ran out of time
        return 1;
    }

    public static String intToColor(int colorVal) {
        if (colorVal == 0) {
            return "Skystone";
        }
        else if (colorVal == 255) {
            return "Stone";
        }
        else return "null";
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. grayscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}