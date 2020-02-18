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


@Autonomous(name="Red_Side_Cubes_Autonomous", group="Pushbot")


public class Red_Side_Cubes_Autonomous extends LinearOpMode {

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
    private double speed = 0.8;
    public static int cubeLocation;
    private double[] distanceRange ;

    @Override

    public void runOpMode() {
/**
 * we init parts from Hardware
 */
        distanceRange = new double[2];
        robot.init(hardwareMap);
        cubeLocation = SkystoneDetectorPhoneCam.position(this,"red");

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
                new Location(620, 0));

      //  waitForStart();
/**
 * we set up the path
 */
        try {

            cubePostion = ((cubeLocation+3) * stoneSize) - 140;

            ad.setPosition(new Location(cubePostion, -600),
                    0, 50, 150, 5, 50, speed ,5000);




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
                    0, 50, 100, 5, 50, speed,2000);


            ad.setPosition(new Location(2000, -200),
                    -90, 100, 100, 5, 30, speed, 5000);


            ///////////////////////////////////////////////////////////////////////////////
            runtime.reset();
            while(runtime.milliseconds() < 500) {
                leftStone.setPower(0.5);
            }
            leftStone.setPower(0);

            ////////////////////////////////////////////////////////////////////////////////////
            cubePostion = ((cubeLocation) * stoneSize)-140;

//            ad.setPosition(new Location(cubePostion, -600),
//                    0, 50, 100, 5, 50, speed, 5000);


            distanceRange[0] = 50 ;
            distanceRange[1] = 100 ;
            ad.setPosition(new Location(cubePostion, -600),
                    0, distanceRange,  150, 5, 50, speed, 150,3000);


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

            ad.setPosition(new Location(cubePostion, -620),
                    0, 50, 100, 5, 50, speed, 5000);


            ad.setPosition(new Location(2000, -100),
                    -90, 100, 100, 5, 30, speed, 2000);
///////


            runtime.reset();
            while(runtime.milliseconds() < 500) {
                leftStone.setPower(0.5);
            }
            leftStone.setPower(0);

            ///

            ad.setPosition(new Location(1800, -200),
                    0, 100, 100, 5, 30, 0.6, 5000);


            ad.stopAllAutoCalculations();
            Teleop.angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - 180;
            stop();
        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace().toString());
        }
        stop();
    }
}
