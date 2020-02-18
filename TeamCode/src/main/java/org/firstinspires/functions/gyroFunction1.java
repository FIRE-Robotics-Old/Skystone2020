package org.firstinspires.functions;

import com.qualcomm.robotcore.util.Range;

public class gyroFunction1 implements gyroProportional{
    double turn = 0 ;
    /**
     * spins the robot in auto with proportion
     * @param angleToReach
     * @param gyroAngle
     * @param slowAngle
     * @param Vmax
     * @return the turn speed
     */
    public double gyroProportionalCalculation(double angleToReach, double gyroAngle, double slowAngle, double Vmax){
        if (slowAngle == 0) return 0;
        turn =   ((angleToReach - gyroAngle)/slowAngle)*Vmax;
        if (Math.abs(turn) < 0.3) {
            if(turn > 0)
                 return 0.3;
            return -0.3;
        }
        Double turnAfterClip = Range.clip(turn ,-Vmax ,Vmax);
        return turnAfterClip;
    }
}
