package org.firstinspires.ftc.teamcode.presets;

import org.firstinspires.ftc.teamcode.Camera;
import org.firstinspires.ftc.teamcode.Robot;

public class ToWobbleZone implements IMovements {
    Robot robot;
    Camera camera;
    int stackSize = 0, inv;
    boolean atAngle = false;

    public ToWobbleZone(Robot robot, Camera camera){
        this.robot = robot;
        this.camera = camera;
    }

    @Override
    public void rotate() {
        if(robot.getRotation() < -1){
            robot.move(0,0,1, .75);
            atAngle = false;
        } else if(robot.getRotation() > 1){
            robot.move(0,0,-1, .75);
            atAngle = false;
        } else {
            robot.stop();
            atAngle = true;
        }
    }

    @Override
    public void path() {
        switch (stackSize) {
            case 0:
                robot.setPos(1000, 1000, 1000, 1000);
                robot.move(1, 0, 0, .75);
                break;
            case 1:
                robot.setPos(1100, 1100, 1100, 1100);
                robot.move(1, 0, 0, .75);
                break;
            case 4:
                robot.setPos(1300, 1300, 1300, 1300);
                robot.move(1, 0, 0, .75);
                break;
        }
    }

    @Override
    public void move() {
        while(!atAngle){
            rotate();
        }

        stackSize = camera.stackSize();

        path();
    }
}
