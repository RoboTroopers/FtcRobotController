package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.AutoTest.State.*;

@Autonomous(name = "Auto Test", group="Autonomous")
public class AutoTest extends AutoBase
{

    protected enum State
    {
        FORWARD,
        BACKWARD,
        LEFT,
        DONE
    }
    
    @Override
    protected int handleState(int currentState, int seconds)
    {
        State state = getCurrentState(State.values());
        telemetry.addData("Current state", state);
        
        switch (getCurrentState(State.values()))
        {
            case FORWARD:
                if (seconds > 2.5) return BACKWARD.ordinal();
                /*glideRelative(0, 50, 0, 0.15, 0, 0.0);
                if (atTarget()) return State.BACKWARD.ordinal();*/
                break;

            case BACKWARD:
                if (seconds > 5) return LEFT.ordinal();
                /*glideRelative(0, -50, 0, 0.5, 0, 0);
                if (atTarget()) return State.LEFT.ordinal();*/
                break;

            case LEFT:
                if (seconds > 7.5) return DONE.ordinal();
                /*glideRelative(0, 0, 0, 0, 90, 0.15);
                if (atTarget()) return State.DONE.ordinal();*/
                break;
        }
        return currentState;
    }
    
}
