package org.firstinspires.ftc.teamcode.infomodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odometry.Kinematics;
import org.firstinspires.ftc.teamcode.odometry.Pathfinder;
import org.firstinspires.ftc.teamcode.odometry.Waypoint;

@Autonomous
public class PathTest extends LinearOpMode {

    Robot robot = new Robot(telemetry);
    Kinematics k = new Kinematics(robot);
    Pathfinder path = new Pathfinder(telemetry);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        path.addWaypoint(new Waypoint(1, 2, 0, 0.8, 2.56 / 100)); // meters
       // path.addWaypoint(new Waypoint(2, 2, 0, 0.4, 2.56 / 100)); // meters
        //path.addWaypoint(new Waypoint(1.5, 1, 0, 0.8, 2.56 / 100)); // meters
        waitForStart();

        path.followPath(k);
        telemetry.addData("/> PATH", "COMPLETED");
        robot.stop();
    }
}
