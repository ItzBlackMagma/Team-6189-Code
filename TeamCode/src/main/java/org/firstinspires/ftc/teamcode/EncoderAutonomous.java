package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class EncoderAutonomous extends LinearOpMode {
    Robot robot = new Robot(telemetry);
    boolean atPosition = false;
    int[] pos = new int[4];

    @Override
    public void runOpMode() throws InterruptedException {

    }

    void runToPosition(double pos1, double pos2, double pos3, double pos4, double power){ // in inches
        robot.setPos(pos1, pos2, pos3, pos4);
        while (!atPosition && opModeIsActive()){
            robot.motor1.setPower(robot.getDirFromPos(robot.getPos()[1], (int) pos1, power));
        }
    }

}
