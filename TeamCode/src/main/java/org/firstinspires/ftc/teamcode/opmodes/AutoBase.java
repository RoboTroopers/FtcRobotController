package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.Bot;

/**
 * Progresses through stages, setting target position and angle of bot depending on the stage
 */

public abstract class AutoBase extends Bot
{
    protected double targetX = 0;
    protected double targetY = 0;
    protected double xSpeed = 0;
    protected double ySpeed = 0;
    
    protected double targetAngle = 0;
    protected double turnSpeed = 0;
    
    protected int currentStage = 0;
    protected boolean stageReady = true;
    
    
    protected void nextStage()
    {
        currentStage++;
        stageReady = true;
    }
    
    protected void startStage()
    {
        stageReady = false;
    }
    
    protected boolean atTargetThreshold(double posErrorThreshold, double angleErrorThreshold)
    {
        return Math.abs(targetX - getWorldXPos()) < posErrorThreshold
            && Math.abs(targetY - getWorldYPos()) < posErrorThreshold
            && Math.abs(targetAngle - getWorldAngle()) < angleErrorThreshold;
    }
    
    protected boolean atTarget()
    {
        return atTargetThreshold(3, 0.1);
    }
    
    protected void moveToTarget() {
        glideToPos(targetX, targetY, xSpeed, ySpeed, targetAngle, turnSpeed);
    }

}
