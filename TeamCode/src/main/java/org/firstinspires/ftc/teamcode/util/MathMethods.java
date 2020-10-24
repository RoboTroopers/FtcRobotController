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

    public static double clamp(double num, double a, double b)
    {
        if (a < b) return Math.max(Math.min(num, b), a);
        else if (a > b) return Math.max(Math.min(num, a), b);
        else return a;
    }
}
