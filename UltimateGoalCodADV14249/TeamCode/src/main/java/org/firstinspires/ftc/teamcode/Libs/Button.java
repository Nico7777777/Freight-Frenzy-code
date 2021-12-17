package org.firstinspires.ftc.teamcode.Libs;

import static java.lang.System.currentTimeMillis;

/**
 * Created by edidi on 06.01.2017.
 */

public class Button {

    public boolean value = false;
    public boolean lastValue = false;
    public boolean toggle = false;
    public int pressed = 0;
    public int toggleInt = 0;
    public boolean intrat = false;
    public long lastToggleTime = currentTimeMillis();
    public void updateValue(boolean value){
        lastValue = this.value;
        this.value = value;
        updateToggle();

    }



    public void updateToggle(){
            if (value == true && lastValue == false) {
                pressed = pressed + 1;
                intrat = true;
                if (toggle == false) {
                    toggle = true;
                    toggleInt = 1;
                    lastToggleTime = currentTimeMillis();
                } else if (toggle == true) {
                    toggle = false;
                    toggleInt = 0;
                }
            }

    }
}
