package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class LaunchRPM extends Thread {

    double timeInterval = 50, CPR = 28, time = 0, lastTime = 0, deltaTime = 0, counts = 0, lastCounts = 0, deltaCounts = 0, rpm = 0, dir = 0, deltaPower = 0;
    RobotHardware robot;
    OpMode opMode;

    @Override
    public void run() {
        if(robot != null && opMode != null)
            getRPM();
    }

    public void init(RobotHardware r, OpMode mode){
        robot = r;
        opMode = mode;
    }

    public double getRPM(){
        time = opMode.getRuntime() / 60; // gets the time in seconds and converts it to minutes
        deltaTime = time - lastTime;

        counts = robot.rightSpin.getCurrentPosition();
        deltaCounts = counts - lastCounts;

        rpm = (deltaCounts / CPR) / deltaTime;
        return rpm;
    }

    public void reset(){
        lastCounts = counts;
        lastTime = time;
    }

    public double setRPM(double desiredRPM, double power, double error){
        dir = desiredRPM - getRPM();


        if(dir > error) {
            power = Math.abs(power);
        } else if (dir < -error) {
            power = 0;
        }

        return power;
    }

}
