package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Pathfinder {
    Telemetry telemetry;

    public Pathfinder(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    private ArrayList<Waypoint> waypoints = new ArrayList<>();

    public void addWaypoint(Waypoint waypoint){
        waypoints.add(waypoint);
    }

    public void addWaypoint(Waypoint waypoint, int index){
        waypoints.add(index, waypoint);
    }

    public void followPath(Kinematics k){
        for (int i = 0; i < waypoints.size(); i++)
            while (!k.atPoint) {
                telemetry.addData("/> POSE X", k.getGLOBAL_X());
                telemetry.addData("/> POSE Y", k.getGLOBAL_Y());
                telemetry.addData("/> POSE R", k.getGLOBAL_R());
                k.goToPosition(waypoints.get(i));
            }
    }

}
