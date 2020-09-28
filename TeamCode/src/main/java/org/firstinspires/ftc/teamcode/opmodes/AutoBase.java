package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.Bot;

/**
 * Progresses through stages, setting target position and angle of bot depending on the stage
 */

public class AutoBase extends Bot
{
    // Distance from target position and angle necessary to move on to next movement stage.
    private static final double posErrorThreshold = 3;
    private static final double angleErrorThreshold = 0.5;

    private double targetX;
    private double targetY;
    private double xSpeed;
    private double ySpeed;

    private double targetAngle;
    private double turnSpeed;

    private int currentStage;


    @Override
    public void init()
    {
        super.init();
    }

    @Override
    public void loop()
    {
        super.loop();
        if (currentStage == 0)
        {
        }

        if (currentStage == 1)
        {
        }

        glideToPos(targetX, targetY, xSpeed, ySpeed, targetAngle, turnSpeed);

        if (Math.abs(targetX - getWorldXPos()) < posErrorThreshold
         && Math.abs(targetY - getWorldYPos()) < posErrorThreshold
         && Math.abs(targetAngle - getWorldAngle()) < angleErrorThreshold)
        {
            currentStage += 1;
        }
    }

}
