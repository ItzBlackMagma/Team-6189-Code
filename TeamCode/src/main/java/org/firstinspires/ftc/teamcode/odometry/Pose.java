package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class Pose {
    private double x = 0;
    private double y = 0;
    private double r = 0;

    public Pose(double x, double y, double r) {
        update(x, y, r);
    }

    public void update(double x, double y, double r) {
        this.setX(this.getX() + x);
        this.setY(this.getY() + y);
        this.setR(r);
    }

    public void setLocation(double x, double y, double r){
        this.setX(x);
        this.setY(y);
        this.setR(r);
    }

    public double getX() {return x;}

    public void setX(double x) {this.x = x;}

    public double getY() {return y;}

    public void setY(double y) {this.y = y;}

    public double getR() {return r;}

    public void setR(double r) {this.r = r;}

    public void printPose(Telemetry telemetry){
        double[] poseInfo = {x,y,r};
        telemetry.addData("/> POSE", Arrays.toString(poseInfo));
    }
}
