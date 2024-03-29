package org.firstinspires;

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


@Autonomous(name="Blue_Side_Cubes_Autonomous", group="Pushbot")


public class Blue_Side_Cubes_Autonomous extends LinearOpMode {

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
    private final int side = -1 ; // blue = -1

    @Override

    public void runOpMode() {
/**
 * we init parts from Hardware
 */
        robot.init(hardwareMap);
        int cubeLocation = SkystoneDetectorPhoneCam.position(this,"blue");

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
                new Location(-620, 0));

      //  waitForStart();
/**
 * we set up the path
 */
        try {
            rightStone.setPower(0);
            cubePostion = ((cubeLocation + 3) * stoneSize)-250;

            ad.setPosition(new Location(side*cubePostion+120 , -600),
                    0, 40, 200, 5, 0, speed);




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
                rightStone.setPower(-0.5);
            }
            rightStone.setPower(0);


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


            ad.setPosition(new Location(side*cubePostion+120, -200),
                    0, 50, 100, 5, 0, speed);


            ad.setPosition(new Location(side*2600, -200),
                    90, 100, 100, 5, 40, speed);

//
//            ///////////////////////////////////////////////////////////////////////////////
//            runtime.reset();
//            while(runtime.milliseconds() < 500) {
//                rightStone.setPower(0.5);
//            }
//
//
//            ////////////////////////////////////////////////////////////////////////////////////
//            cubePostion = ((cubeLocation) * stoneSize-250);
//
//            ad.setPosition(new Location(side*cubePostion+120, -600),
//                    0, 50, 200, 5, 30, speed);
//
//
//
//
//            runtime = new ElapsedTime();
//            while(runtime.milliseconds() < 800)
//            {
//                leftSide.setPower(0.5);
//                rightSide.setPower(0.5);
//            }
//
//
//            leftSide.setPower(0);
//            rightSide.setPower(0);
//
//            runtime.reset();
//            while(runtime.milliseconds() < 1000) {
//                rightStone.setPower(-0.5);
//            }
//            rightStone.setPower(0);
//
//            ad.setPosition(new Location(side*cubePostion+120, -200),
//                    0, 50, 100, 5, 0, speed);



            ad.setPosition(new Location(side*2400, -100),
                    90, 100, 100, 5, 40, speed);
///////


            runtime.reset();
            while(runtime.milliseconds() < 500) {
                rightStone.setPower(0.5);
            }
            rightStone.setPower(0);

            ///

            ad.setPosition(new Location(side*1800, -200),
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
            stop();
        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace().toString());
        }
        //stop();
    }
}
