package org.firstinspires;
 /**
 * we import imu ,LinearOpMode,DcMotor,distance sensor,ElapsedTime, Telemetry,AngleUnit,AxesOrder,AxesReference,DistanceUnit,driveFunction1,driveProportional,gyroFunction1 and gyroProportional
 */

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ActiveLocation;
import org.firstinspires.Location;
import org.firstinspires.functions.driveFunction1;
import org.firstinspires.functions.driveProportional;
import org.firstinspires.functions.gyroFunction1;
import org.firstinspires.functions.gyroProportional;

public class AutoDrivingSecondTry {
    /** Declare  the parts and verbals   */

        private DcMotor leftSide;
        private DcMotor rightSide;
        private DcMotor middleMotor;
        private BNO055IMU imu;
        private DistanceSensor sideDistanceSensor;
        private DistanceSensor frontDistanceSensor;
        private Telemetry telemetry;
        private ElapsedTime runtime = new ElapsedTime();
        private DistanceToTargetFinder distanceToTargetFinder;
        private ActiveLocation activeLocation;
        private driveProportional driveProportional;
        private gyroProportional gyroProportional;
        private Thread targetLocationThread;
        private Thread currentLocationThread;
        private LinearOpMode linearOpMode;
        public boolean threadsStopped = false;

    /**
     * the constructor
     * @param leftSide
     * @param rightSide
     * @param middleMotor
     * @param imu
     * @param telemetry
     * @param linearOpMode
     */
        public AutoDrivingSecondTry (DcMotor leftSide, DcMotor rightSide, DcMotor middleMotor,
                           BNO055IMU imu/*, Field field */, Telemetry telemetry , LinearOpMode linearOpMode,
                                     DistanceSensor sideDistanceSensor, DistanceSensor frontDistanceSensor, Location beginningPoint)
        {

            this.linearOpMode = linearOpMode;
            driveProportional = new driveFunction1();
            gyroProportional = new gyroFunction1();
            this.leftSide = leftSide;
            this.rightSide = rightSide;
            this.middleMotor = middleMotor;
            this.imu = imu;
            this.telemetry = telemetry;
            this.sideDistanceSensor = sideDistanceSensor;
            this.frontDistanceSensor = frontDistanceSensor;

            activeLocation = new ActiveLocation(leftSide, rightSide, middleMotor, imu , beginningPoint);
            currentLocationThread = new Thread(activeLocation);
            currentLocationThread.start();

            distanceToTargetFinder = new DistanceToTargetFinder(activeLocation, imu);
            targetLocationThread = new Thread(distanceToTargetFinder);
            targetLocationThread.start();
        }

    /**
     * stop the threads
     */
        public void stopAllAutoCalculations(){
            distanceToTargetFinder.setStop(true);
            activeLocation.setStop(true);
            threadsStopped = true;
        }

        public void updateYAxis(double y)
        {
            activeLocation.setY_Axis(y);
        }

    public void updateXAxis(double x)
    {
        activeLocation.setX_Axis(x);
    }

    /**
     * allows to move the foundation and update the location
     */
    public void DriveToWall(double angleToReach , double angleRange , double slowAngle , double Vmax, double distanceFromWall)
        {
            double gyroAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double gyroPropVal = gyroProportional.gyroProportionalCalculation(angleToReach, gyroAngle, slowAngle, Vmax/2);


            runtime.reset();
            while(frontDistanceSensor.getDistance(DistanceUnit.MM) > distanceFromWall &&
                    !Thread.interrupted() && linearOpMode.opModeIsActive())
            {
                Log.e("CUBES", "positionning infront of a cube will start now");
                leftSide.setPower(-0.8 - gyroPropVal);
                rightSide.setPower(-0.8 + gyroPropVal);
                telemetry.addData("distance: ", frontDistanceSensor.getDistance(DistanceUnit.MM));
                telemetry.update();
            }

            leftSide.setPower(0);
            rightSide.setPower(0);
            telemetry.addData("point:","point(%.2f,%.2f)", activeLocation.getX_Axis(), activeLocation.getY_Axis());

            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 200){}

            activeLocation.updateCurrentLocationWithSensors(frontDistanceSensor.getDistance(DistanceUnit.MM));
                     // sideDistanceSensor.getDistance(DistanceUnit.MM));

        }

    public static double convertToDistanceX(double ticks){
        double     DRIVE_GEAR_REDUCTION    = 20 ;
        double     WHEEL_DIAMETER_MM   = 90 ;
        return (ticks * WHEEL_DIAMETER_MM * Math.PI) / (DRIVE_GEAR_REDUCTION * 28.5);
    }


    public static double convertToDistanceY(double ticks) {
        double DRIVE_GEAR_REDUCTION = 20;
        double WHEEL_DIAMETER_MM = 90;
        return -((ticks * WHEEL_DIAMETER_MM * Math.PI) / (DRIVE_GEAR_REDUCTION * 28.5));
    }




    /**
     * calculates the speed and the spinning speed and combines bet ween them
     * @param locationToReach
     * @param angleToReach
     * @param distanceRange
     * @param slowDrive
     * @param angleRange
     * @param slowAngle
     * @param Vmax
     */
        public void setPosition(Location locationToReach , double angleToReach , double distanceRange , double slowDrive , double angleRange , double slowAngle , double Vmax){
            try {
                telemetry.addData("Begining is Active still alive: ", currentLocationThread.isAlive());
                telemetry.addData("Begining is finder still alive:", targetLocationThread.isAlive());
                telemetry.update();

//            ElapsedTime runtime = new ElapsedTime();
//            while (runtime.time() < 2000){}

                distanceToTargetFinder.setNewPoint(locationToReach);

                double[] distancesToDrive = distanceToTargetFinder.getDistanceTotarget();
                double gyroAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double rightMotorSpeed;
                double leftMotorSpeed;
                double middleMotorSpeed;
                while (!Thread.interrupted() && linearOpMode.opModeIsActive() && !(Math.abs(distancesToDrive[0]) < distanceRange && Math.abs(distancesToDrive[1]) < distanceRange
                        && Math.abs(angleToReach-gyroAngle) < angleRange))
                {
                    distancesToDrive = distanceToTargetFinder.getDistanceTotarget();
                    telemetry.addData("encoders: left ",leftSide.getCurrentPosition());
                    telemetry.addData("encoders: right ",rightSide.getCurrentPosition());
                    telemetry.addData("encoders: midddle ",middleMotor.getCurrentPosition());
                    telemetry.addData("x axis distance: ", distancesToDrive[0]);
                    telemetry.addData("y axis distance: ", distancesToDrive[1]);
                    telemetry.addData("x current: ", activeLocation.getX_Axis());
                    telemetry.addData("y current: ", activeLocation.getY_Axis());
                    telemetry.addData("chosen cube:", SkystoneDetectorPhoneCam.cubeLocation);

                    gyroAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                    double DrivePropValStraight = driveProportional.driveProportionalFunction(distancesToDrive[1], slowDrive , Vmax);
                    double DrivePropValMiddle = driveProportional.driveProportionalFunction(distancesToDrive[0], slowDrive , Vmax);
                    double gyroPropVal = gyroProportional.gyroProportionalCalculation(angleToReach, gyroAngle, slowAngle, Vmax/2);
                    double Max = Math.max(Math.abs(distancesToDrive[0]), Math.abs(distancesToDrive[1]));
                    double driveProp = distancesToDrive[1] * (-DrivePropValStraight);
                    double firstCalc = 1 - Math.abs(gyroPropVal);

                    Max = Math.max(Max ,1.0 );
                    rightMotorSpeed = ((firstCalc) * (driveProp) / Max) + gyroPropVal;
                    leftMotorSpeed = ((firstCalc) * (driveProp) / Max) - gyroPropVal;
                    middleMotorSpeed = distancesToDrive[0] * DrivePropValMiddle / Max;

                    telemetry.addData("left speed ", leftMotorSpeed);
                    telemetry.addData("middle speed: ", middleMotorSpeed);
                    telemetry.addData("right speed: ", rightMotorSpeed);
                    telemetry.addData("gyro angle: ", gyroAngle);
                    telemetry.addData("first driveprop caculation: ", driveProp);
                    telemetry.addData("first 1-gyroPropVal caculation: ", firstCalc);
                    telemetry.update();

                    leftSide.setPower(leftMotorSpeed);
                    rightSide.setPower(rightMotorSpeed);
                    middleMotor.setPower(middleMotorSpeed);
                 //   Thread.sleep(50);
                }


                distancesToDrive = distanceToTargetFinder.getDistanceTotarget();
//                telemetry.addData("encoders: left ",leftSide.getCurrentPosition());
//                telemetry.addData("encoders: right ",rightSide.getCurrentPosition());
//                telemetry.addData("encoders: midddle ",middleMotor.getCurrentPosition());

                telemetry.addData("x current: ", activeLocation.getX_Axis());
                telemetry.addData("y current: ", activeLocation.getY_Axis());
                telemetry.addData("After while exit :x axis distance: ", distancesToDrive[0]);
                telemetry.addData("After while exit :y axis distance: ", distancesToDrive[1]);
                telemetry.addData("is Active still alive: ", currentLocationThread.isAlive());
                telemetry.addData("is finder still alive:", targetLocationThread.isAlive());

                telemetry.update();
                leftSide.setPower(0);
                rightSide.setPower(0);
                middleMotor.setPower(0);
//                runtime = new ElapsedTime();
//                while (runtime.time() < 10000) {
//                }
            }catch (Exception e){ telemetry.addData("error:",e.getStackTrace().toString());}

        }

}
