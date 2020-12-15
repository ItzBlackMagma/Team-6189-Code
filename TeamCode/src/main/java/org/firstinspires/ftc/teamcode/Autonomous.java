package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Autonomous extends LinearOpMode {

    RobotHardware robot = new RobotHardware(telemetry);
    Camera camera = new Camera(robot);
    Crossbow bow = new Crossbow(robot, camera);
    RingDetector detector = new RingDetector(camera, telemetry);

    enum StartPosition { RED_WALL, RED_MID, BLUE_WALL, BLUE_MID }
    enum Team {RED, BLUE}

    StartPosition startPos;
    Team team;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        camera.activate(hardwareMap);
        detector.initTfod(hardwareMap);

        camera.track();
        detector.detect();

        wait(100);

        // get team
        if (robot.robotY > 0){
            team = Team.BLUE;
        } else {
            team = Team.RED;
        }
        
    }
}
