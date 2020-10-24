package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.State;

import java.util.Timer;

/**
 * Robot progresses through states, setting target position and angle of bot depending on the stage
 */

public abstract class AutoBase extends OpmodeBase
{

    /**
     * State management
     */

    private int currentState = 0;
    private final ElapsedTime timer = new ElapsedTime();
    
    protected <T extends Enum> T getCurrentState(T[] states)
    {
        for (T e: states)
        {
            if (e.ordinal() == currentState)
                return e;
        }
        
        telemetry.addData("State int not matched", currentState);
        return null;
    }

    protected abstract int handleState(int currentState, int ms);

    @Override
    public void loop()
    {
        super.loop();
        int lastState = currentState;

        currentState = handleState(currentState, (int) timer.milliseconds());
        
        if (currentState != lastState)
        {
            timer.reset();
        }
    }
    
    
    /**
     * Movement management
     */
    
    protected double targetX = 0;
    protected double targetY = 0;
    protected double xSpeed = 0;
    protected double ySpeed = 0;
    
    protected double targetAngle = 0;
    protected double turnSpeed = 0;
    
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
    
    protected void glideToTarget()
    {
        glideToPos(targetX, targetY, xSpeed, ySpeed, targetAngle, turnSpeed);
    }
    
}
