package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestAuto extends LinearOpMode {

    RobotHardware robot = new RobotHardware(telemetry);
    Camera camera = new Camera(robot);
    Crossbow bow = new Crossbow(robot, camera);
    RingDetector detector = new RingDetector(camera, telemetry);

    Team team;

    int stackSize;
    int inv = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // init phase
        robot.init(hardwareMap);
        camera.activate(hardwareMap);
        detector.initTfod(hardwareMap);

        for (int i = 0; i < 20; i ++) {
            camera.track();
            stackSize = detector.detect();
        }

        team = robot.getTeam(camera);

        switch (team){
            case RED:
                inv = -1;
                break;
            case BLUE:
                inv = 1;
                break;
        }

        waitForStart();
        // run phase
        while (!robot.atPoint) {
            robot.moveToPoint(camera.quadField, camera.quadField / 2, .5, 1);
        }

        robot.stop();

    }

}
