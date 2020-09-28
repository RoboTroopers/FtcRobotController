package org.firstinspires.ftc.teamcode.util;

public class MathMethods
{

    // Keep angle within range -180 to 180 degrees while preserving the angle.
    public static double angleWrap(double angle)
    {
        while (angle < -180)
        {
            angle += 360;
        }
        while (angle > 180)
        {
            angle -= 360;
        }
        return angle;
    }

    public static double clamp(double num, double min, double max)
    {
        return Math.max(Math.min(num, max), min);
    }
}
