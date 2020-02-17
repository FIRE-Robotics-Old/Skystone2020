package org.firstinspires;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Location;


@Autonomous(name="Red_Side_Cubes_Autonomous", group="Pushbot")


public class Red_Side_Cubes_Autonomous extends SkystoneDetectorPhoneCam {

    /**
     * Declare on the parts
     */
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private Servo fundationHolder;
    private Servo gripperServo;
    private DistanceSensor frontDistanceSensor;
    private DistanceSensor sideDistanceSensor;
    private Servo leftExpantion;
    private DcMotor leftSide;
    private DcMotor rightSide;
    private DcMotor middleMotor;
    private DcMotor gripperMotor;
    private DcMotor rightStone;
    private DcMotor leftStone;
    private double stoneSize = 203.2;
    private int mode = 1;
    private double fieldSide = 3660;
    private double robotLength = 450;
    private double robotWidth = 430;
    private double robotWidthWhenOpen = 550;
    private double cubePostion = 0;
    private double speed = 0.6;

    @Override

    public void runOpMode() {
/**
 * we init parts from Hardware
 */
        robot.init(hardwareMap);
        int cubeLocation = 1;
        try{
            super.runOpMode();
            cubeLocation = super.cubeLocation;
        } catch (Exception e)
        {

        }
        imu = robot.imu;
        fundationHolder = robot.fundationHolder;
        sideDistanceSensor = robot.sideDistanceSensor;
        frontDistanceSensor = robot.frontDistanceSensor;
        leftExpantion = robot.leftExpantion;
        leftSide = robot.leftDrive;
        rightSide = robot.rightDrive;
        middleMotor = robot.middleDrive;
        gripperMotor = robot.gripperMotor;
        gripperServo = robot.gripperServo;
        leftStone = robot.left_stone;
        rightStone = robot.right_stone;
        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        AutoDriving AD = new AutoDriving(robot.rightDrive , robot.leftDrive , robot.middleDrive ,
//                 robot.imu,rp,field , telemetry);
/**
 * we send motors imu and distance sensors
 */
        AutoDrivingSecondTry ad = new AutoDrivingSecondTry(robot.leftDrive, robot.rightDrive, robot.middleDrive,
                robot.imu, telemetry, this, frontDistanceSensor, sideDistanceSensor,
                new Location(600, 0));

      //  waitForStart();
/**
 * we set up the path
 */
        try {

            cubePostion = ((cubeLocation) * stoneSize) - 140;

            ad.setPosition(new Location(cubePostion, -600),
                    0, 50, 100, 5, 50, speed);




            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 700)
            {
                leftSide.setPower(0.5);
                rightSide.setPower(0.5);
            }


            leftSide.setPower(0);
            rightSide.setPower(0);


//            gripperServo.setPosition(0.1);

//            gripperMotor.setPower(1);
            runtime.reset();
            while(runtime.milliseconds() < 1000) {
                leftStone.setPower(-0.5);
            }
            leftStone.setPower(0);



//            runtime.reset();
//            while(runtime.milliseconds() < 1500)
//            {
//                leftSide.setPower(-0.3);
//                rightSide.setPower(-0.3);
//            }
//
//            leftSide.setPower(0);
//            rightSide.setPower(0);
//            gripperMotor.setPower(0);


            runtime.reset();
            while(runtime.milliseconds() < 1500) {
            }


            ad.setPosition(new Location(cubePostion, -200),
                    0, 50, 100, 5, 50, speed);


            ad.setPosition(new Location(2000, -200),
                    -90, 100, 100, 5, 30, speed);


            ///////////////////////////////////////////////////////////////////////////////
            runtime.reset();
            while(runtime.milliseconds() < 500) {
                leftStone.setPower(0.5);
            }
            leftStone.setPower(0);

            ////////////////////////////////////////////////////////////////////////////////////
            cubePostion = ((cubeLocation+3) * stoneSize)-100;

            ad.setPosition(new Location(cubePostion, -600),
                    0, 70, 100, 5, 50, speed);




            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 800)
            {
                leftSide.setPower(0.5);
                rightSide.setPower(0.5);
            }


            leftSide.setPower(0);
            rightSide.setPower(0);

            runtime.reset();
            while(runtime.milliseconds() < 1000) {
                leftStone.setPower(-0.5);
            }
            leftStone.setPower(0);

            ad.setPosition(new Location(cubePostion, -200),
                    0, 50, 100, 5, 50, speed);


            ad.setPosition(new Location(2400, -100),
                    -90, 100, 100, 5, 30, speed);
///////


            runtime.reset();
            while(runtime.milliseconds() < 500) {
                leftStone.setPower(0.5);
            }
            leftStone.setPower(0);

            ///

            ad.setPosition(new Location(1800, -200),
                    0, 100, 100, 5, 30, 0.6);

//            runtime.reset();
//            while(runtime.milliseconds() < 1500) {
//            }

//            runtime.reset();
//            while(runtime.milliseconds() < 300) {
//                gripperMotor.setPower(-0.8);
//            }
        //    gripperMotor.setPower(0);

//            runtime.reset();
//            while(runtime.milliseconds() < 1500) {
//            }

//            ad.setPosition(new Location(0, 1000), 0, 50, 200, 5, 40, 0.6);
//
//            cubePostion = (mode * stoneSize) + robotLength/2;
//
//            ad.setPosition(new Location(600, cubePostion), 120, 20, 200, 5, 30, 0.3);
//
//            runtime.reset();
//            while(runtime.milliseconds() < 500)
//            {
//                gripperMotor.setPower(0.8);
//                leftSide.setPower(-0.4);
//                rightSide.setPower(-0.4);
//            }
//
//            leftSide.setPower(0);
//            rightSide.setPower(0);
//            gripperMotor.setPower(0);
//
//            ad.setPosition(new Location(0, 1000), 0,
//                    50, 200, 5, 40, 0.6);
//
//            ad.setPosition(new Location(0, 1800), 0,
//                    50, 200, 5, 40, 0.8);
//
//            runtime.reset();
//            while(runtime.milliseconds() < 300) {
//                gripperMotor.setPower(-0.8);
//            }
//            gripperMotor.setPower(0);
            ad.stopAllAutoCalculations();
            Teleop.angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - 180;
        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace().toString());
        }
    }
}
