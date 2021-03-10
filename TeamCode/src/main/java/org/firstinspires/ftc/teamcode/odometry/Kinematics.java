package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.Robot;

public class Kinematics extends Thread {
    private Robot robot;
    private Pose pose = new Pose(0,0,0);

    long sleepTime = 50; // in milliseconds
    public final double CPR = 28, CPI = 45, R = 4; // R is the radius of the wheel
    private double xPos = 0, yPos = 0, rotation = 0;
    private double motor1, motor2, motor3, motor4, xVelocity, yVelocity, rVelocity, resultantVelocity, xDis, yDis, theta;
    private double GLOBAL_X = 0, GLOBAL_Y = 0, PREV_X = 0, PREV_Y = 0, GLOBAL_R;
    private int lastM1 = 0, lastM2 = 0, lastM3 = 0, lastM4 = 0;

    public boolean isRunning = true, atPoint = false;

    public Kinematics(Robot robot){
        this.robot = robot;
    }
    public Kinematics(Robot robot, Pose startingPose){
        this.robot = robot;
        this.pose = startingPose;
    }

    public void run(){
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
//        xDis = xVelocity * (sleepTime * 1000);
//        yDis = yVelocity * (sleepTime * 1000);
//        theta = (Math.PI / 2) - robot.getRotation(); // gets the angle of the components
//
//        // turns the robot relative vectors to field oriented
//        GLOBAL_X = (xDis * Math.sin(theta)) + (yDis * Math.cos(theta));
//        GLOBAL_Y = (xDis * Math.cos(theta)) + (yDis * Math.sin(theta));

        GLOBAL_Y = ((yVelocity * Math.cos(robot.getRotation())) + (xVelocity * Math.sin(robot.getRotation()))) * (sleepTime * 1000);
        GLOBAL_X = ((yVelocity * Math.sin(robot.getRotation())) + (xVelocity * Math.cos(robot.getRotation()))) * (sleepTime * 1000);
        GLOBAL_R = robot.getRotation();

        // getPose().update(GLOBAL_X - PREV_X, GLOBAL_Y - PREV_Y, GLOBAL_R);
        pose.setLocation(GLOBAL_X, GLOBAL_Y,GLOBAL_R);

        PREV_X = GLOBAL_X;
        PREV_Y = GLOBAL_Y;
    }

    private void updateRobotVelocity(){
        // gets wheel velocities
        motor1 = getMotorSpeed(robot.motor1.getCurrentPosition() - lastM1);
        motor2 = getMotorSpeed(robot.motor2.getCurrentPosition() - lastM2);
        motor3 = getMotorSpeed(robot.motor3.getCurrentPosition() - lastM3);
        motor4 = getMotorSpeed(robot.motor4.getCurrentPosition() - lastM4);

        // calculates the robot's velocities in x, y and r relative to the robot
        yVelocity = (motor1 + motor2 + motor3 + motor4) * (R/4); // longitudinal component (forward)
        xVelocity = (-motor1 + motor2 + motor3 - motor4) * (R/4); // transversal component (sideways)
        rVelocity = (-motor1 + motor2 - motor3 + motor4) * (R/4); // angular component

        resultantVelocity = xVelocity + yVelocity + rVelocity; // the final velocity of the robot will be the sum of each component

        // reset the last motor positions
        lastM1 = robot.motor1.getCurrentPosition();
        lastM2 = robot.motor2.getCurrentPosition();
        lastM3 = robot.motor3.getCurrentPosition();
        lastM4 = robot.motor4.getCurrentPosition();
    }

    private double getMotorSpeed(int pos){ // returns meters per second
        return (pos * (1 / CPI)) / (sleepTime * 1000);
    }

    public void goToPosition(double x, double y, double angRad, double speed, double error){
        double absoluteAngle = Math.atan2(y - GLOBAL_Y, x - GLOBAL_X);
        double relativeAngle  = angleWrap(absoluteAngle - robot.getRotation());

        double distance = Math.hypot(x - GLOBAL_X, y - GLOBAL_Y);

        double relativeX = Math.cos(relativeAngle) * distance;
        double relativeY = Math.sin(relativeAngle) * distance;

        double v = Math.abs(relativeX) + Math.abs(relativeY);
        double xPower = relativeX / v;
        double yPower = relativeY / v;

        if (Math.abs(distance) > error) atPoint = false; else atPoint = true;

        if (!atPoint) robot.move(xPower, yPower, 0, speed);
    }

    public void goToPosition(double x, double y, double speed, double error){
        goToPosition(x,y,GLOBAL_R,speed,error);
    }

    public void goToPosition(Waypoint w){
        goToPosition(w.getX(), w.getY(), w.getSpeed(), w.getError());
    }

    public double rotateToAngle(double angRad){
        angRad = angleWrap(angRad - GLOBAL_R);
        if(angRad > 0.2) return 1;
        if(angRad < 0.2) return -1;
        return 0;
    }

    public double angleWrap(double angle){
        while(angle < -Math.PI) angle += 2*Math.PI;
        while(angle > Math.PI) angle -= 2*Math.PI;
        return angle;
    }

    public double getGLOBAL_X() {
        return GLOBAL_X;
    }

    public double getGLOBAL_Y() {
        return GLOBAL_Y;
    }

    public double getGLOBAL_R() {
        return GLOBAL_R;
    }

    public Pose getPose() {
        return pose;
    }
}
