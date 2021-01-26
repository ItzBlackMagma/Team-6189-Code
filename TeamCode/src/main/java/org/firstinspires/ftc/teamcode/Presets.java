package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.presets.IMovements;
import org.firstinspires.ftc.teamcode.presets.ToWobbleZone;

import java.util.ArrayList;

public class Presets {
    private Robot robot;

    int index = 0;
    ArrayList<IMovements> presets = new ArrayList<>();

    public Presets(Robot robot){
        this.robot = robot;

        presets.add(new ToWobbleZone(robot, robot.camera));
    }

    public void runNextPreset(){
        presets.get(index).move();
        index++;
    }



}
