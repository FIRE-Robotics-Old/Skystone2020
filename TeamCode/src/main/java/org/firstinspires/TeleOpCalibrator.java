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

package org.firstinspires;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TeleOp Calibrator", group="AutonomousTest")
public class TeleOpCalibrator extends LinearOpMode {


    /**
     *     Declare parts from the Hardware class
     *     and create variable
     */

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftSide;
    private DcMotor gripperMotor;
    private DcMotor rightSide;
    private DcMotor middleMotor;
    private DcMotor liftMotor;

    private BNO055IMU imu;
    private DistanceSensor liftSensor;
    private Servo gripperServo;
    private Servo fundationHolder;
    volatile boolean g2StartButtonIsPressed = false;
    volatile boolean g1StartButtonIsPressed = false;
    volatile boolean g1BackButtonIsPressed = false;
    volatile boolean isClawPressed;
    volatile boolean isGripperPressed;
    volatile boolean isFourbarPressed;
    volatile boolean isLiftDownPressed;
    volatile boolean isLiftUpPressed;
    boolean rollerGriperIsOpen = true;
    boolean fourbarIsOpen = true;
    boolean graberIsOpen = true;
    boolean driveToPosition = false;
    public static double angle;
    double powerY;
    double powerX;
    double gyroAngle;
    double leftPower;
    double rightPower;
    double middlePower;



    @Override
    public void runOpMode() {
        /**
 * init the components
 */
        robot.init(hardwareMap);
        liftMotor = robot.liftMotor;
        leftSide = robot.leftDrive;
        rightSide = robot.rightDrive;
        middleMotor = robot.middleDrive;
        gripperMotor = robot.gripperMotor;
        liftSensor = robot.liftSensor;
        gripperServo = robot.gripperServo;
        fundationHolder = robot.fundationHolder;
        imu = robot.imu;
        //   parkingMotor = robot.parkingMotor;

/**
 * create more variables
 */
        boolean manualControl = false;
        boolean fieldOrientatedDrive = false;
        boolean foundationHolderIsOpened = false;
        driveToPosition = false;
        int level = 0;
        gyroAngle = 0;
        isClawPressed = false;
        isFourbarPressed = false;
        isGripperPressed = false;
        isLiftDownPressed = false;
        isLiftUpPressed = false;

        ActiveLocation al = new ActiveLocation(leftSide, rightSide, middleMotor, imu, new Location(-620, 0));
        al.oppositeCalculation();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
/** create/init  the threads for the teleop
 *
 */



/**
 * sets the mode for the motors
 */
        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        al.run();

        while (opModeIsActive()) {
            /**
             * switch between of the drive train (field Orientated Drive and normal)
             */
            if (gamepad1.back && fieldOrientatedDrive && !g1BackButtonIsPressed) {
                fieldOrientatedDrive = false;
                g1BackButtonIsPressed = true;
            } else if (gamepad1.back && !fieldOrientatedDrive && !g1BackButtonIsPressed) {
                fieldOrientatedDrive = true;
                g1BackButtonIsPressed = true;
            } else if(!gamepad1.back){
                g1BackButtonIsPressed = false;
            }

            gyroAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + angle;
            /**
             * the different calculations of the  different modes
             */
            if (fieldOrientatedDrive) {
                gyroAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + angle;
                powerY = -(gamepad1.left_stick_x * Math.sin(Math.toRadians(-gyroAngle))) + gamepad1.left_stick_y * (Math.cos(Math.toRadians(-gyroAngle)));
                powerX = gamepad1.left_stick_x * Math.cos(Math.toRadians(-gyroAngle)) + gamepad1.left_stick_y * (Math.sin(Math.toRadians(-gyroAngle)));
                rightPower = leftPower = powerY;
                middlePower = powerX;
            } else {
                rightPower = leftPower = gamepad1.left_stick_y;
                middlePower = gamepad1.left_stick_x;
            }
/**
 *  turning to both sids
 */
            if (gamepad1.left_trigger > 0) {
                rightPower = -gamepad1.left_trigger;
                leftPower = gamepad1.left_trigger;
                rightPower = Range.clip(rightPower, -0.6, 0.6);
                leftPower = Range.clip(leftPower, -0.6, 0.6);
            } else if (gamepad1.right_trigger > 0) {
                rightPower = gamepad1.right_trigger;
                leftPower = -gamepad1.right_trigger;
                rightPower = Range.clip(rightPower, -0.6, 0.6);
                leftPower = Range.clip(leftPower, -0.6, 0.6);
            }
/**
 * sets power
 */
            leftSide.setPower(leftPower);
            rightSide.setPower(rightPower);
            middleMotor.setPower(middlePower);
/**
 * the opening and closing fourbar of the claw
 */
            if (gamepad2.a && !fourbarIsOpen && !isFourbarPressed) {
                robot.fourbarServo.setPosition(0);
                fourbarIsOpen = true;
                isFourbarPressed = true;
            } else if (gamepad2.a && fourbarIsOpen && !isFourbarPressed) {
                robot.fourbarServo.setPosition(1);
                fourbarIsOpen = false;
                isFourbarPressed = true;
            }    else if (!gamepad2.a) {
                isFourbarPressed = false;
            }


/**
 * switch between the different  mods of the lift (levels and manual)
 */
            if (gamepad2.start && !g2StartButtonIsPressed && !manualControl) {
                manualControl = true;
                g2StartButtonIsPressed = true;
            } else if (gamepad2.start && !g2StartButtonIsPressed && manualControl) {
                manualControl = false;
                g2StartButtonIsPressed = true;
            } else if(!gamepad2.start){
                g2StartButtonIsPressed = false;
            }

/**
 * move the lift up
 */
            if (gamepad2.dpad_up && level < 6 && !isLiftUpPressed && !manualControl) {
                level++;
                isLiftUpPressed = true;
                driveToPosition = true;
            } else if (gamepad2.dpad_up && !isLiftUpPressed && manualControl) {
                isLiftUpPressed = true;
                liftMotor.setPower(1);
            } else if (!gamepad2.dpad_up) {
                isLiftUpPressed = false;
            }

/**
 * move the  lift down
 */
            if (gamepad2.dpad_down && level > 0 && !isLiftDownPressed && !manualControl) {
                level--;
                isLiftDownPressed = true;
                driveToPosition = true;
            } else if (gamepad2.dpad_down && manualControl) {
                liftMotor.setPower(-1);
            } else if (!gamepad2.dpad_down) {
                isLiftDownPressed = false;
            }

/**
 * resets the lift
 */
            if (gamepad2.b && !manualControl) {
                level = 0;
                driveToPosition = true;
            }


            if(driveToPosition)
            {
                liftAutoControl(manualControl, level);
            }

/**
 * stops the lift
 */
            if (manualControl && !gamepad2.dpad_down && !gamepad2.dpad_up)
            {
                liftMotor.setPower(0);
                driveToPosition = true;
            }


/**
 * opening and closing the expantion
 */
            if (gamepad1.left_bumper) {
                robot.rightExpantion.setPosition(0);
                robot.leftExpantion.setPosition(1);
            } else if (gamepad1.right_bumper) {
                robot.leftExpantion.setPosition(0);
                robot.rightExpantion.setPosition(1);
            }

/**
 *opening and closing the roller griper
 */
            if (gamepad2.x && rollerGriperIsOpen && !isGripperPressed) {
                gripperServo.setPosition(1);//close
                rollerGriperIsOpen = false;
                isGripperPressed = true;
            } else if (gamepad2.x && !rollerGriperIsOpen && !isGripperPressed) {
                gripperServo.setPosition(0.45);
                rollerGriperIsOpen = true;
                isGripperPressed = true;
            } else if (!gamepad2.x) {
                isGripperPressed = false;
            }

/**
 * spine the intake
 */
            if (gamepad2.right_bumper) {
                gripperMotor.setPower(0.8);
            } else if (gamepad2.left_bumper) {
                gripperMotor.setPower(-0.8);
            } else {
                gripperMotor.setPower(0);
            }

/**
 * open and close the fundationHolder
 */
            if (gamepad1.start && !foundationHolderIsOpened && !g1StartButtonIsPressed) {
                fundationHolder.setPosition(0);
                foundationHolderIsOpened = true;
                g1StartButtonIsPressed = true;
            } else if (gamepad1.start && foundationHolderIsOpened && !g1StartButtonIsPressed) {
                fundationHolder.setPosition(1);
                foundationHolderIsOpened = false;
                g1StartButtonIsPressed = true;
            } else {
                g1StartButtonIsPressed = false;
            }
/**
 * open and close the claw
 */
            if (gamepad2.y && graberIsOpen && !isClawPressed) {
                robot.clawServo.setPosition(0.36);//close
                graberIsOpen = false;
                isClawPressed = true;
            } else if (gamepad2.y && graberIsOpen == false && !isClawPressed) {
                robot.clawServo.setPosition(0.6);//open
                graberIsOpen = true;
                isClawPressed = true;
            } else if (!gamepad2.y) {
                isClawPressed = false;
            }
/**
 * open and close the parkingMotor
 */
            telemetry.addData("X", al.getX_Axis());
            telemetry.addData("Y", al.getY_Axis());
            telemetry.update();
        }
    }

    /**
     * move the lift between levels
     * @param manualControl
     * @param level
     */

    public void liftAutoControl(boolean manualControl, int level)
    {
        int range = 15;
        double minimalHight = 24;
        double foundationHight = minimalHight + 50;
        double level_height = 100;
        double sample;
        double error;

        if (!manualControl)
        {
            if (level == 0)
            {
                sample = liftSensor.getDistance(DistanceUnit.MM) - minimalHight;
            }
            else
            {
                sample = liftSensor.getDistance(DistanceUnit.MM) - (foundationHight + level_height * (level - 1));
            }

            error = Math.abs(sample);

            if (error > range)
            {
                if (sample < 0)
                {
                    liftMotor.setPower(1);
                }
                else if (sample > 0)
                {
                    liftMotor.setPower(-1);
                }
            }
            else
            {
                liftMotor.setPower(0);
                driveToPosition = false;
            }
        }
    }

    /**
     * convert distance to ticks (x axis)
     * @param distance
     * @return ticks in the robot x axis
     */
    public static double convertToTicksX(double distance){
        double     DRIVE_GEAR_REDUCTION    = 20 ;
        double     WHEEL_DIAMETER_MM   = 90 ;
        return ((distance * DRIVE_GEAR_REDUCTION  * 28.5)/ (WHEEL_DIAMETER_MM * Math.PI ));
    }

    /**
     * convert distance to ticks (y axis)
     * @param distance
     * @return ticks in the robot y axis
     */
    private double convertToTicksY(double distance){
        double     DRIVE_GEAR_REDUCTION    = 20 ;
        double     WHEEL_DIAMETER_MM   = 90 ;
        return -((distance * DRIVE_GEAR_REDUCTION  * 28.5)/ (WHEEL_DIAMETER_MM * Math.PI));
    }

}