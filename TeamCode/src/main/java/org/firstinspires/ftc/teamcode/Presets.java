package org.firstinspires.ftc.teamcode;

public class Presets {
    Robot robot;

    int preset = 0;

    public Presets(Robot robot){
        this.robot = robot;
    }

    public void runNextPreset(){
        preset++;

        switch (preset){
            case 0:
                //do stuff
                break;
            case 1:
                //do stuff
                break;
            case 2:
                //do stuff
                break;
            case 3:
                //do stuff
                break;
        }
    }



}
