package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;

public class Pathfinder {
    private Telemetry telemetry;
    private ArrayList<Waypoint> waypoints = new ArrayList<>();

    public Pathfinder(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void addWaypoint(Waypoint waypoint){
        waypoints.add(waypoint);
    }

    public void addWaypoint(Waypoint waypoint, int index){
        waypoints.add(index, waypoint);
    }

    public void followPath(Kinematics k){
        for (int i = 0; i < waypoints.size(); i++)
            while (!k.atPoint) {
                k.getPose().printPose(telemetry);
                telemetry.update();
                k.goToPosition(waypoints.get(i));
            }
    }

}
