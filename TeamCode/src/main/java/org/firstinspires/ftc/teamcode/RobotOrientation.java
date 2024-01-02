package org.firstinspires.ftc.teamcode;

public class RobotOrientation {
    public static double angle = 0;
    public static boolean red = true;

    public static double getAngle()
    {
        if(red)
        {
            return -angle;
        }
        else
        {
            return angle + 180;
        }
    }
}
