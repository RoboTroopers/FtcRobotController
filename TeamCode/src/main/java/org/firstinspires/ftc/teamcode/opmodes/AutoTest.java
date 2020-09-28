package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Test", group="Autonomous")
public class AutoTest extends AutoBase
{
    @Override
    public void loop()
    {
        super.loop();
        
        if (currentStage == 0)
        {
            if (stageReady)
            {
                startStage();
                targetX = 0;
                targetY = 100;
                xSpeed = 0.1;
                ySpeed = 0.1;
            }
            moveToTarget();
            
            if (atTarget())
            {
                nextStage();
            }
        }

        if (currentStage == 1)
        {
            if (stageReady)
            {
                startStage();
                targetX = 150;
                targetY = 200;
                xSpeed = 0.1;
                ySpeed = 0.1;
            }
            moveToTarget();

            if (atTarget())
            {
                nextStage();
            }
        }
    }
    
}
