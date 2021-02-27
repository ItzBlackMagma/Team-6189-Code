package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class EncoderAutonomous extends LinearOpMode {
    Robot robot = new Robot(telemetry);
    boolean atPosition = false;
    int[] pos = new int[4];

    int stackSize = 1;
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
        telemetry.addData("/> STACK SIZE", stackSize);
        telemetry.update();
        stackSize = 1;
        sleep(100);

        moveToWobbleZone();
        goal = 2;
        runToPosition(goal, goal, goal, goal, 0.8);
        sleep(waitTime - 1000);
        telemetry.addData("/>", "MOVING TO OTHER START LINE");
        telemetry.update();
        double offset = Locations.LINE_2 * .75;
        strafe(goal, offset, 0.8);
        sleep(2000);
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
        sleep(4000);

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
                double offset = 16;
                waitTime = 6000;
                strafe(goal, offset,0.8);
                break;
            case 4:
                goal = Locations.WOBBLE_ZONE_C - 18;
                waitTime = 7000;
                setWheelPower(.8,.8,.8,.8);
                robot.setPos(goal,goal,goal,goal);
                robot.toPosition();
                break;
        } sleep(waitTime);
    }

    void runToPosition(double pos1, double pos2, double pos3, double pos4, double power){ // in inches
        robot.move(0,1,0,power);
        robot.setPos(pos1, pos2, pos3, pos4);
        robot.toPosition();
    }

    void setWheelPower(double p1, double p2, double p3, double p4){
        robot.motor1.setPower(p1);
        robot.motor2.setPower(p2);
        robot.motor3.setPower(p3);
        robot.motor4.setPower(p4);
    }

    void strafe(double pos, double offset, double power){
        robot.move(1,0,0,power);
        robot.setPos(pos - offset,pos + offset,pos - offset,pos + offset);
        robot.toPosition();
    }

    public void setLoaderPower(double power, long ms){
        robot.launcher.load(-power);
        sleep(ms);
        robot.launcher.load(0);
    }

}
