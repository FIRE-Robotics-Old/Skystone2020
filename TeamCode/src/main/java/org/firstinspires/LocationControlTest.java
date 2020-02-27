package org.firstinspires;


/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.Location;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import java.util.Observable;
//import java.util.Observer;


@Autonomous(name="LocationControlTest", group="Pushbot")

public class LocationControlTest extends LinearOpMode  {

    /** Declare on the parts  */
    HardwareUS robot = new HardwareUS();
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private Servo fundationHolder;
    private DistanceSensor frontDistanceSensor;
    private DistanceSensor sideDistanceSensor;
    private DcMotor leftSide;
    private DcMotor rightSide;
    private DcMotor middleMotor;
    public double powerFactor=0.8;
    public final int side = -1 ; // blue -1
    double[] distanceRange;

    private static AutoDrivingSecondTry ad;
    //private ActiveLocation al; //changed, as well as commented out lne below
    @Override

    public void runOpMode() {
        //       al = new ActiveLocation(leftSide, rightSide, middleMotor, imu, new Location(-620, 0));
        /*
         * we init parts from Hardware
         */
        distanceRange = new double[2];

        robot.init(hardwareMap);

        imu = robot.imu;
        //fundationHolder = robot.fundationHolder;
        //sideDistanceSensor = robot.sideDistanceSensor;
        //frontDistanceSensor = robot.frontDistanceSensor;
        leftSide = robot.leftDrive;
        rightSide = robot.rightDrive;
        middleMotor = robot.middleDrive;

        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




/**
 * we send motors imu and distance sensors
 */

        waitForStart();
/**
 *
 * we set up the path
 */

        try {
            if(ad == null || ad.threadsStopped) {
                ad = new AutoDrivingSecondTry(robot.leftDrive, robot.rightDrive, robot.middleDrive,
                        robot.imu, telemetry, this, frontDistanceSensor, sideDistanceSensor, new Location(-620,0));
            }
            runtime = new ElapsedTime(); //added
            while(runtime.milliseconds()<500){}//added

            distanceRange[0]= 70;
            distanceRange[1] = 70;//added
            ad.setPosition(new Location(side*350, side*800),
                    0, distanceRange, 200, 5, 30, powerFactor,200);




            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 200)
            {
                leftSide.setPower(powerFactor);
                rightSide.setPower(powerFactor);
            }


            leftSide.setPower(0);
            rightSide.setPower(0);


            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 500){ }


            //fundationHolder.setPosition(0);


            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 1000){ }


            ad.DriveToWall(0*side, 5, 30, powerFactor, 250);

            //test
            distanceRange[0] = 150;
            distanceRange[1] = 150; //Added
            ad.setPosition(new Location(side*600,side*600 ) ,-90, distanceRange ,300 ,5 ,10 ,powerFactor,200, 7000); //Change to 700?
            //    ad.setPosition(new Location(700, 400), 0 , 50 ,300 ,10 ,20 ,0.5);



            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 2000)
            {
                leftSide.setPower(1);
                rightSide.setPower(1);
            }

            leftSide.setPower(0);
            rightSide.setPower(0);

            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 500){} //changed from 1500 to 500

            ad.updateXAxis(-465);

            //fundationHolder.setPosition(1);

            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 500){}


//             telemetry.addData(" loc " , ad.);
            distanceRange[0] = 100;
            distanceRange[1] = 100;
            telemetry.addData("Moved", "false");
            ad.setPosition(new Location(side*1900, 500*side),
                    -90, distanceRange, 200, 10, 0, 1, 200, 5000);
            telemetry.addData("Moved", "true");
            telemetry.update();
            //fundationHolder.setPosition(0); //changed

            runtime.reset();
            while (runtime.milliseconds() < 1000)
                middleMotor.setPower(-1);
            middleMotor.setPower(0);

            //  ad.stopAllAutoCalculations();
            Teleop.angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - 180;
            stop();
        }
        catch (Exception e)
        { telemetry.addData("error:",e.getStackTrace()/*.toString()*/);//changed
            ad.stopAllAutoCalculations();}
    }

}


//Robot_Prameters rp = new Robot_Prameters( 2500 , new Location(0,0)); // I assumed we use dimensions in millimeters
        //Field field  = new Field(400000 , 40000);
        //imu = robot.imu;
        //fundationHolder = robot.fundationHolder;



//
//        AutoDriving AD = new AutoDriving(robot.rightDrive , robot.leftDrive , robot.middleDrive ,
//                 robot.imu,rp,field , telemetry);

        /*AutoDrivingSecondTry ad = new AutoDrivingSecondTry(leftSide, rightSide, middleMotor, imu, telemetry,this, null,null, new Location(0,0));
        // robot.imu, telemetry, this);

        waitForStart();
        while (opModeIsActive()) {

            try {
                //rightExpansion.setPosition(0.67);
                Location one;
                Location two;
                //camera vision
                int at = 1;

                Location merge = new Location(-456,0);


                if (at==1){
                    one = new Location(-1115,709);
                    two = new Location(-1115,608);
                }else if (at == 2){
                    one = new Location(-1100, -456);
                    two = new Location (-1100,-355);//Moves four in to secure block
                }else{
                    one = new Location(-1064,304);
                    two = new Location(-1064,203);
                }


                ad.setPosition(one,
                        0, 25, 25, 10, 30, 0.3);
                //runtime = new ElapsedTime();
                //telemetry.speak("At position one");

                //telemetry.speak(angle);
                telemetry.update();
                leftSide.setPower(0);
                rightSide.setPower(0);
                double start = runtime.milliseconds();
                //while (runtime.milliseconds()-start < 1000)  { telemetry.addData("time:",runtime.milliseconds()-start ); telemetry.update();}
                //fundationHolder.setPosition(0);
                //Thread.sleep(3000);
                start = runtime.milliseconds();
                while (runtime.milliseconds() - start < 1000) {
                    telemetry.addData("time:", runtime.milliseconds() - start);
                    telemetry.addData("angle: ", imu.getAngularOrientation());
                    telemetry.update();
                }
                //String angle = ""+imu.getAngularOrientation();


                //Thread.sleep(1000);
                ad.setPosition(two,
                        0, 25, 25, 10, 30, 0.3 );

                //start = runtime.milliseconds() ;
                //while (runtime.milliseconds()-start < 1000) { }
                Thread.sleep(1000);
                //telemetry.addData("At pos: ", 2);
                //telemetry.update();
                //fundationHolder.setPosition(1);
                //start = runtime.milliseconds() ;
                //while (runtime.milliseconds()-start < 1000) { }
                //Thread.sleep(1000);

                //y-=300;
                // ad.setPosition(new Location(-152, -101),
                //0, 25, 20, 10, 30, 0.5 );
                //ad.setPosition(new Location(1500, (y)), 0, 50, 25, 10, 0, 0.5 );
                //ad.setPosition(new Location(1500, y),
                //180,25,20,10,30, 0.5);


                ad.setPosition(merge,
                        0, 25, 20, 10, 30, 0.5 );
                ad.setPosition(new Location(-456, 400),
                        0, 25, 25, 10, 0, 0.5 );
                //rightExpansion.setPosition(0.9);
                ad.setPosition(new Location(-456, 400),
                        -90, 25, 25, 10, 0, 0.5 );
                //rightExpansion.setPosition(.64);

                ad.stopAllAutoCalculations();
                stop();
            } catch (Exception e) {
                telemetry.addData("error:", e.getStackTrace().toString());
            }
            //runtime = new ElapsedTime();

            //while (runtime.milliseconds() < 10000) { }

//        Location point = new Location( 830 , 420);
//        AD.DriveToCordinate(point , 15 , 0 );
//
//        AD.RotateRobotToAngle(180, 3, 90 ,0.6);
//
//        point = new Location( 500 , 980);
//        AD.DriveToCordinate(point , 15 , 180 );
//
//        fundationHolder.setPosition(1);
//
//        point = new Location( 830 , 220);
//        AD.DriveToCordinate(point , 15 , 180 );
//
//        fundationHolder.setPosition(0);

//        Location point = new Location( -150 , -300);
//        AD.DriveToCordinate(point , 15 , 0 );

//        runtime.reset();
//        while(runtime.milliseconds() < 1000){}
//
//
//        AD.RotateRobotToAngle(-90,4,20,0.6);
//
//
//        point = new Location( -150 , -400);
//        AD.DriveToCordinate(point , 15 , -90 );
//
//
//        runtime.reset();
//        while(runtime.milliseconds() < 1000){}
//
//
//        robot.rightExpantion.setPosition(0);
//
//
//        runtime.reset();
//        while(runtime.milliseconds() < 1000){}
//
//
//        AD.RotateRobotToAngle(0,3,15,1);
//
//        point = new Location( -600 , -400);
//        AD.DriveToCordinate(point , 20 , 0 );
//
//        robot.rightExpantion.setPosition(1);
//
//
//        point = new Location( 1000 , 0);
//        AD.DriveToCordinate(point , 50 , 0 );

            stop();
        }
    }
}
*/


