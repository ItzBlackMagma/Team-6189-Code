package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class EncoderAutonomous extends LinearOpMode {
    Robot robot = new Robot(telemetry);
    boolean atPosition = false;
    int[] pos = new int[4];

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.resetEncoders();
        robot.stop();
        robot.setPos(0, 0, 0, 0);
        robot.toPosition();
        waitForStart();

        runToPosition(12,12,12,12,0.8);
        sleep(5000);

        /*
        int stackSize = robot.camera.stackSize();
        int goal = 0;
        long waitTime = 7000;
        switch (stackSize) {
            case 0:
                goal = 3000;
                waitTime = 4000;
                runToPosition(goal,goal,goal,goal,0.8);
                sleep(waitTime);
                break;
            case 1:
                goal = 3700;
                waitTime = 6000;
                runToPosition(goal,goal,goal,goal,0.8);
                sleep(waitTime);
                break;
            case 4:
                goal = 5000;
                waitTime = 7000;
                runToPosition(goal,goal,goal,goal,0.8);
                sleep(waitTime);
                break;
        }

        goal = 2700;
        runToPosition(goal, goal, goal, goal, -.8);
        sleep(waitTime);

        robot.launcher.launch(0.62);
        sleep(2000);
        setLoaderPower(1,4000);

        goal = 3000;
        runToPosition(goal,goal,goal,goal,0.8);
        sleep(10000);
         */

    }

    void runToPosition(double pos1, double pos2, double pos3, double pos4, double power){ // in inches
        robot.move(0,1,0,power);
        robot.setPos(pos1, pos2, pos3, pos4);
        robot.toPosition();
    }

    public void setLoaderPower(double power, long ms){
        robot.launcher.load(-power);
        sleep(ms);
        robot.launcher.load(0);
    }

}
