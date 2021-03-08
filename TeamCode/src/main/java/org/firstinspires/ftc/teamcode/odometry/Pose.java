package org.firstinspires.ftc.teamcode.odometry;

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

    public static double imuToDegree(double angle){
        return ((angle * 2) + 180);
    }

    public double getX() {return x;}

    public void setX(double x) {this.x = x;}

    public double getY() {return y;}

    public void setY(double y) {this.y = y;}

    public double getR() {return r;}

    public void setR(double r) {this.r = r;}
}
