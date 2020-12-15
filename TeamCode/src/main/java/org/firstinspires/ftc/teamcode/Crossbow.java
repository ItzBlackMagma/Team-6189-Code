package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Crossbow {

    RobotHardware robot;
    Camera camera;
    Telemetry telemetry;

    // target locations | +X = forward, +Y = left, +Z = up
    public double[] highGoal = {0,0,0}, // all targets/goals are set based off the origin in the center of the playing field
                    target1 = {0,0,0}, // target closest to tower
                    target2 = {0,0,0}, // middle target
                    target3 = {0,0,0}; // target farthest from tower

    public void redTeam(){ // inverses the Y position of each target to allow for the red side
        highGoal[1] *= -1;
        target1[1] *= -1;
        target2[1] *= -1;
        target3[1] *= -1;
    }


    public Crossbow(RobotHardware robot, Camera camera) {
        this.robot = robot;
        this.telemetry = robot.telemetry;
        this.camera = camera;
    }

    public void init(){

        //odometry stuff
        //robot.globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, robot.COUNTS_PER_INCH, 75);
        //Thread positionThread = new Thread(robot.globalPositionUpdate);
        //positionThread.start();

        //robot.globalPositionUpdate.reverseRightEncoder();
        //robot.globalPositionUpdate.reverseNormalEncoder();
    }
//
//    public void aim(double[] target, double power){
//        double desiredUpAngle = calculateUp(target);
//        double desiredSideAngle = calculateSide(target);
//
//        int upTicks = (int) ((desiredUpAngle * robot.COUNTS_PER_REV) / 360);
//        int sideTicks = (int)((desiredSideAngle * robot.COUNTS_PER_REV) / 360);
//
//        /**
//        robot.upwards.setTargetPosition(upTicks);
//        while (robot.upwards.getCurrentPosition() < upTicks)
//
//        robot.sideways.setTargetPosition(sideTicks);
//
//        robot.upwards.setPower();
//         **/
//    }

    public void turnHorizontal(double power){ robot.sideways.setPower(power); }

    public void turnVertical(double power){ robot.vertical.setPower(power); }

    public void drawBack() throws InterruptedException {
        int restPosition = 0;
        int backPosition = 2;

        robot.drawback.setTargetPosition(backPosition);
        while (robot.drawback.getCurrentPosition() < backPosition)
            robot.drawback.setPower(1);
        robot.drawback.setPower(0);

        lock();
        wait(500);

        robot.drawback.setTargetPosition(restPosition);
        while (robot.drawback.getTargetPosition() > restPosition)
            robot.drawback.setPower(-1);
        robot.drawback.setPower(0);
    }

    public void lock(){ robot.lock.setPosition(1); }

    public void fire(){
        robot.lock.setPosition(-1);
    }
//
//    public double calculateUp(double[] target){
//        double robotX = robot.globalPositionUpdate.returnXCoordinate();
//        double robotY = robot.globalPositionUpdate.returnYCoordinate();
//        double distance = Math.hypot(robotX - target[0], robotY - target[1]);
//
//        double theta = Math.atan((target[2] - robot.launchHeight) / distance);
//        theta = (360 * robot.vertical.getCurrentPosition()) / robot.COUNTS_PER_REV;
//        double gravityCushion = calculateGravityCushion(distance, 1, theta);
//
//        theta = Math.atan(((target[1] - robot.launchHeight) + (gravityCushion * -1)) / distance);
//
//        return theta;
//    }
//
//    public double calculateSide(double[] target){
//        double robotX = robot.globalPositionUpdate.returnXCoordinate();
//        double robotY = robot.globalPositionUpdate.returnYCoordinate();
//        double distance = Math.hypot(robotX - target[0], robotY - target[1]);
//
//        double adjacentSide = Math.hypot(robotX - target[0], robotY - robotY);
//        double theta = Math.acos(adjacentSide / distance);
//        return theta;
//    }

    public double calculateGravityCushion(double distance, double velocity, double theta){
        double t = distance / (velocity * Math.cos(theta));
        double y = (velocity * Math.sin(theta)) * t + (.5 * -9.81 * Math.pow(t, 2));
        return y / 50;
    }

}
