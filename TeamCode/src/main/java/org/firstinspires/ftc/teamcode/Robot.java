package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.zip.CheckedOutputStream;

public class Robot {

    // Modules
    public Camera camera;
    public Wobble wobble;
    public Launcher launcher;
    public BNO055IMU imu;

    private Telemetry telemetry;
    public DcMotor motor1, motor2, motor3, motor4; // starts with the front left and moves clockwise

    public static final double COUNTS_PER_INCH = 5; // needs to be replaced with the actual number

    public Robot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Takes in x, y, r, and power to make the robot move in any direction
     * @param x The power of the robot forward(+) and back(-)
     * @param y The power of the robot right(+) and left(-)
     * @param r The power of the robot in rotating to the right(+) or to the left(-)
     * @param power The speed control to easily adjust how fast the robot will move
     */
    public void move(double x, double y, double r, double power){

        double power1 = y + x + r;
        double power2 = y - x - r;
        double power3 = y + x - r;
        double power4 = y - x + r;

        motor1.setPower(power1 * power);
        motor2.setPower(power2 * power);
        motor3.setPower(power3 * power);
        motor4.setPower(power4 * power);
    }

    /**
     * Takes in the desired target position using inches, then it converts the inches to usable encoder counts.
     * This makes it easier to make mesurements.
     * @param pos1 Target position of motor1
     * @param pos2 Target position of motor2
     * @param pos3 Target position of motor3
     * @param pos4 Target position of motor4
     */
    public void setPos(double pos1, double pos2, double pos3, double pos4){
        motor1.setTargetPosition((int) (pos1 / COUNTS_PER_INCH));
        motor2.setTargetPosition((int) (pos2 / COUNTS_PER_INCH));
        motor3.setTargetPosition((int) (pos3 / COUNTS_PER_INCH));
        motor4.setTargetPosition((int) (pos4 / COUNTS_PER_INCH));
    }

    public void init(HardwareMap hardwareMap){
        camera = new Camera(hardwareMap, telemetry);
        wobble = new Wobble();
        launcher = new Launcher();

        wobble.init(hardwareMap);
        launcher.init(hardwareMap);
        camera.activate(hardwareMap);

        initIMU(hardwareMap);
        initMotors(hardwareMap);
    }

    private void initIMU(HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("/> INIT", "Imu Initialized");
    }

    private void initMotors(HardwareMap hardwareMap){
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("/> INIT", "Chassis Motors Initialized");
    }

    public void noEncoders(){
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void toPosition(){
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoders(){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
