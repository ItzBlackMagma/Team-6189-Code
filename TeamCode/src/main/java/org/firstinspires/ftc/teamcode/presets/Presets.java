package org.firstinspires.ftc.teamcode.presets;

import android.sax.StartElementListener;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.presets.IMovements;
import org.firstinspires.ftc.teamcode.presets.ToWobbleZoneA;
import org.firstinspires.ftc.teamcode.presets.ToWobbleZoneB;
import org.firstinspires.ftc.teamcode.presets.ToWobbleZoneC;

import java.util.ArrayList;

public class Presets {
    private Robot robot;

    int index = 0;
    int stackSize = 0;
    ArrayList<IMovements> presets = new ArrayList<>();

    public Presets(Robot robot){
        this.robot = robot;

        presets.add(new ToWobbleZoneA(robot));
        presets.add(new ToWobbleZoneB(robot));
        presets.add(new ToWobbleZoneC(robot));
    }

    public void runNextPreset(){
        if (index == presets.size() - 1) {
            presets.get(index).move();
            index++;
        } else {
            robot.telemetry.addData("/> PRESETS ",  "FINISHED LAST PRESET (" + index + ")");
        }
    }

    public void runWobbleSpecificPreset(int stackSize){
        this.stackSize = stackSize;
        int i = 0;
        switch (stackSize) {
            case 0:
                i = presets.indexOf(new ToWobbleZoneA(robot));
                break;
            case 1:
                i = presets.indexOf(new ToWobbleZoneB(robot));
                break;
            case 4:
                i = presets.indexOf(new ToWobbleZoneC(robot));
        }
        presets.get(i).move();
        index = i + 1;
    }

}
