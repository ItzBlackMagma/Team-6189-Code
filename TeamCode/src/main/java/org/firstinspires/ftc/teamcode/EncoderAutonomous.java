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

        int stacksize = robot.camera.stackSize();
        int goal = 0;
        long waitTime = 7000;
        switch (stacksize) {
            case 0:
                goal = 3000;
                waitTime = 4000;
                break;
            case 1:
                goal = 3700;
                waitTime = 6000;
                break;
            case 4:
                goal = 5000;
                waitTime = 7000;
                break;
        }



        runToPosition(goal, goal, goal, goal, .8);
        sleep(waitTime);
        goal = 2700;
        runToPosition(goal, goal, goal, goal, -.8);
        sleep(waitTime);

    }

    void runToPosition(double pos1, double pos2, double pos3, double pos4, double power){ // in inches
       // robot.setPos(pos1, pos2, pos3, pos4);
        telemetry.update();
        robot.stop();

//            atPosition = true;
//            if(robot.motor1.getCurrentPosition() == pos1) {
//                robot.motor1.setPower(robot.getDirFromPos(robot.getPos()[0], (int) pos1, power));
//                atPosition = false;
//            }
//            if (robot.motor2.getCurrentPosition() == pos2) {
//                robot.motor2.setPower(robot.getDirFromPos(robot.getPos()[1], (int) pos2, power));
//                atPosition = false;
//            }
//            if (robot.motor3.getCurrentPosition() == pos3) {
//                robot.motor3.setPower(robot.getDirFromPos(robot.getPos()[2], (int) pos3, power));
//                atPosition = false;
//            }
//            if (robot.motor4.getCurrentPosition() == pos4) {
//                robot.motor4.setPower(robot.getDirFromPos(robot.getPos()[3], (int) pos4, power));
//                atPosition = false;
//            }

        robot.move(0,1,0,.5);
        robot.setPos(pos1, pos2, pos3, pos4);
        robot.toPosition();
    }

}
