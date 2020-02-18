package org.firstinspires;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Skystone Autonomous", group = "Autonomous")
public class RedSkystone extends LinearOpMode {
    static int skystone;
    @Override
    public void runOpMode() {
        skystone = SkystoneDetectorPhoneCam.position(this, "red");
        telemetry.addData("Skystone", + skystone);
        telemetry.speak("The Skystone is in position " + skystone);
        telemetry.update();
    }
}
