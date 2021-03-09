package org.firstinspires.ftc.teamcode.odometry;

import java.util.ArrayList;

public class Pathfinder {

    private ArrayList<Waypoint> waypoints = new ArrayList<>();

    public void addWaypoint(Waypoint waypoint){
        waypoints.add(waypoint);
    }

    public void addWaypoint(Waypoint waypoint, int index){
        waypoints.add(index, waypoint);
    }

    public void followPath(Kinematics k){
        for (int i = 0; i < waypoints.size(); i++)
            while (!k.atPoint) k.goToPosition(waypoints.get(i));
    }

}
