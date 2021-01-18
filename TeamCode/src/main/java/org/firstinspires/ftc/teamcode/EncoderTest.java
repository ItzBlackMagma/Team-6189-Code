package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EncoderTest extends OpMode {

    DcMotor motor;
    double time = 0, lastTime = 0, pos = 0, lastPositon = 0, CPR = 29, deltaPos = 0, deltaTime = 0, rpm = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "right spin");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        time = getRuntime();
        pos = motor.getCurrentPosition();
        motor.setPower(-gamepad2.right_stick_y);
        deltaPos = pos - lastPositon;
        deltaTime = time - lastTime;

        rpm = (double) ((deltaPos / CPR) / (deltaTime / 60));

        telemetry.addData("/> RPM", rpm);
        telemetry.addData("/> Delta Time", time - lastTime);
        telemetry.addData("/> Time", time);
        telemetry.addData("/> Last Time", lastTime);
        telemetry.addData("/> POS", pos);
        telemetry.addData("/> Last Pos", lastPositon);
        telemetry.addData("/> Delta pos", pos - lastPositon);
        telemetry.update();
        lastTime = time;
        lastPositon = pos;
    }
}
