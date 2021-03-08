package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.Robot;

public class Kinematics extends Thread {
    private Robot robot;
    public Pose pose = new Pose(0,0,0);

    long sleepTime = 50; // in milliseconds
    public final double CPR = 28, CPI = 45, R = 4; // R is the radius of the wheel
    private double xPos = 0, yPos = 0, rotation = 0;
    private double motor1, motor2, motor3, motor4, xVelocity, yVelocity, rVelocity, resultantVelocity, xDis, yDis, theta;
    private double GLOBAL_X = 0, GLOBAL_Y = 0, PREV_X = 0, PREV_Y = 0;

    public boolean isRunning = true;

    public Kinematics(Robot robot){
        this.robot = robot;
    }

    public void Run(){
        while (isRunning) {
            updateRobotVelocity();
            updatePosition();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void updatePosition(){
        xDis = xVelocity * (sleepTime * 1000);
        yDis = yVelocity * (sleepTime * 1000);
        theta = 90 - robot.getRotation(); // gets the angle of the components

        // turns the robot relative vectors to field oriented
        GLOBAL_X = (xDis * Math.sin(90 - theta)) + (yDis * Math.cos(90 - theta));
        GLOBAL_Y = (xDis * Math.cos(90 - theta)) + (yDis * Math.sin(90 - theta));

        pose.update(GLOBAL_X - PREV_X, GLOBAL_Y - PREV_Y, robot.getRotation());
    }

    private void updateRobotVelocity(){
        // gets wheel velocities
        motor1 = getMotorSpeed(robot.motor1.getCurrentPosition());
        motor2 = getMotorSpeed(robot.motor2.getCurrentPosition());
        motor3 = getMotorSpeed(robot.motor3.getCurrentPosition());
        motor4 = getMotorSpeed(robot.motor4.getCurrentPosition());

        // calculates the robot's velocities in x, y and r relative to the robot
        xVelocity = (motor1 + motor2 + motor3 + motor4) * (R/4); // longitudinal component
        yVelocity = (-motor1 + motor2 + motor3 - motor4) * (R/4); // transversal component
        rVelocity = (-motor1 + motor2 - motor3 + motor4) * (R/4); // angular component

        resultantVelocity = xVelocity + yVelocity + rVelocity; // the final velocity of the robot will be the sum of each component
    }

    private double getMotorSpeed(int pos){ // returns meters per second
        return (double) ((pos * (1 / CPI)) / (sleepTime * 1000));
    }

}
