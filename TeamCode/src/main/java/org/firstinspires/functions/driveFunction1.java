package org.firstinspires.functions;

import com.qualcomm.robotcore.util.Range;

public class driveFunction1 implements  driveProportional{

    private double minVal=0.4;
    /**
     * drives with proportion in auto
     * @param distance
     * @param slow
     * @param vmax
     * @return motor value with proportion
     */
    public double driveProportionalFunction(double distance, double slow, double vmax){
        if(slow == 0) return 0;
        double incline = 0.7 / slow;
        double motorValue = incline * distance + 0.3;

        if (Math.abs(motorValue) < minVal) {
            return 0.3;
        }

        return Math.abs(Range.clip(motorValue,-vmax,vmax));
    }
}
