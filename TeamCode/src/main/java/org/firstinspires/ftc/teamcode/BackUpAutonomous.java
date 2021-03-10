package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BackUpAutonomous extends LinearOpMode {
    Robot robot = new Robot(telemetry);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.resetEncoders();
        waitForStart();

        moveToPoint(Locations.LAUNCH_LINE, 4000);
        moveToPoint(-20, 2000);
        robot.launcher.spinPower(0.7);
        sleep(5000);
        for (int i = 0; i < 3; i++) {
            robot.launcher.fire();
            sleep(300);
            robot.launcher.reload();
        }

        robot.launcher.spinPower(0);
        sleep(4000);
        moveToPoint(20, 2000);
        robot.stop();
    }

    void moveToPoint(double pos, double offset, double xPower, double yPower, long sleep){
        robot.move(xPower, yPower,0,1);
        robot.setPos(pos - offset,pos + offset,pos - offset,pos + offset);
        robot.toPosition();
        sleep(sleep);
        robot.resetEncoders();
    }

    void moveToPoint(double pos, long sleep){moveToPoint(pos, 0, 0, 0.8, sleep);} // forward

}
