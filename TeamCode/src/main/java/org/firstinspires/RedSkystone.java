package org.firstinspires;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Skystone Autonomous", group = "Autonomous")
public class RedSkystone extends LinearOpMode {
    static int skystone;
    Hardware Robot = new Hardware();
    @Override
    public void runOpMode() {
        int skystoneMovedTicks = 0;
        Robot.init(hardwareMap);
        skystone = SkystoneDetectorPhoneCam.position(this, "red");
        telemetry.addData("Skystone", + skystone);
        telemetry.speak("The Skystone is in position " + skystone);
        telemetry.update();
        AutoDrivingSecondTry autonomous = new AutoDrivingSecondTry(
                Robot.leftDrive, Robot.rightDrive, Robot.middleDrive, Robot.imu, telemetry,
                this, Robot.sideDistanceSensor, Robot.frontDistanceSensor,
                new Location(0, 0)
        );

        autonomous.move(20 * 25.4, -6 * 25.44);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds() < 10000){}
    }
}
