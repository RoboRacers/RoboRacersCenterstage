package org.firstinspires.ftc.teamcode.autonomous;

public class AutoMith {
    boolean finished = false;
    boolean propfound = false;
    private enum state{
        DRIVE_TO_PROP,
        LOCATE_PROP,
        PLACE_PIXEL_ON_SPIKE,
        LOCATE_BD,
        PLACE_PIXEL_ON_BD,
        LOCATE_STACK1,
        LOCATE_STACK2,
        LOCATE_STACK3,
        GO_TO_STACK1,
        GO_TO_STACK2,
        GO_TO_STACK3,
        PICKUP_PIXEL,
    }

     void findprop(){
        if (propfound = false){
            final state locateProp = state.LOCATE_PROP;
            final state placepixelonspike = state.PLACE_PIXEL_ON_SPIKE;
        }
        if()
    }
}
