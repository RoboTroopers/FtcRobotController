package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.AutoTest.State.*;

@Autonomous(name = "Auto Test", group="Autonomous")
public class AutoTest extends AutoBase
{
    
    enum State
    {
        FORWARD,
        BACKWARD,
        LEFT,
        DONE
    }
    
    @Override
    protected int handleState(int currentState, int ms)
    {
        telemetry.addData("State", currentState);
        
        switch (getCurrentState(State.values()))
        {
            case FORWARD:
                if (ms > 2500) return BACKWARD.ordinal();
                /*glideRelative(0, 50, 0, 0.15, 0, 0.0);
                if (atTarget()) return State.BACKWARD.ordinal();*/
                break;

            case BACKWARD:
                if (ms > 5000) return LEFT.ordinal();
                /*glideRelative(0, -50, 0, 0.5, 0, 0);
                if (atTarget()) return State.LEFT.ordinal();*/
                break;

            case LEFT:
                if (ms > 7500) return DONE.ordinal();
                /*glideRelative(0, 0, 0, 0, 90, 0.15);
                if (atTarget()) return State.DONE.ordinal();*/
                break;
        }
        return currentState;
    }
    
}
