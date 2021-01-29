package org.firstinspires.ftc.teamcode.presets;

import org.firstinspires.ftc.teamcode.Robot;

public class ToWobbleZoneC implements IMovements {
    Robot robot;
    boolean atAngle = false;

    public ToWobbleZoneC(Robot robotv){
        this.robot = robot;
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
        robot.setPos(127.25, 127.25, 127.25, 127.25);
        robot.move(1, 0, 0, .75);
    }

    @Override
    public void move() {
        robot.noEncoders();
        while(!atAngle){
            rotate();
        }

        robot.resetEncoders();
        robot.toPosition();
        path();
    }

}