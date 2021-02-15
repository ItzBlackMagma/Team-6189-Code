package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class EncoderAutonomous extends LinearOpMode {
    Robot robot = new Robot(telemetry);
    boolean atPosition = false;
    int[] pos = new int[4];

    int stackSize = robot.camera.stackSize();
    double goal;
    long waitTime = 7000;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.resetEncoders();
        robot.stop();
        robot.setPos(0, 0, 0, 0);
        robot.toPosition();
        waitForStart();

       // runToPosition(12,12,12,12,0.8);
       // sleep(5000);

        stackSize = robot.camera.stackSize();
        sleep(100);

        moveToWobbleZone();
        goal = Locations.LINE_2;
        runToPosition(goal, goal, goal, goal, 0.8);
        sleep(waitTime - 1000);
        moveToWobbleZone();

        goal = Locations.STARTER_STACK_Y + 5;
        runToPosition(goal, goal, goal, goal, 0.8);
        sleep(waitTime);

        robot.launcher.launch(0.70);
        sleep(2000);
        setLoaderPower(1,4000);
        robot.launcher.launch(0);

        goal = 10;
        runToPosition(goal,goal,goal,goal,0.8);
        sleep(10000);

    }

    void moveToWobbleZone(){
        switch (stackSize) {
            case 0:
                goal = Locations.WOBBLE_ZONE_A - 18;
                waitTime = 4000;
                runToPosition(goal,goal,goal,goal,0.8);
                break;
            case 1:
                goal = Locations.WOBBLE_ZONE_B - 18;
                double offset = Locations.WOBBLE_ZONE_B_OFFSET;
                waitTime = 6000;
                runToPosition(goal - offset, goal + offset, goal - offset, goal + offset,0.8);
                break;
            case 4:
                goal = Locations.WOBBLE_ZONE_C - 18;
                waitTime = 7000;
                runToPosition(goal,goal,goal,goal,0.8);
                break;
        } sleep(waitTime);
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
