//   ******Front******
//   * 0           2 *
//   *               *
//   *               *
//   *               *
//   * 1           3 *
//   ******Back*******

// Motor 0 --- Pin 0 --- motorFrontLeft
// Motor 1 --- Pin 1 --- motorBackLeft
// Motor 2 --- Pin 2 --- motorFrontRight
// Motor 3 --- Pin 3 --- motorBackRight

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp

public class PracticeDrive extends OpMode
{
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor Arm = null;
    private DcMotorEx Alien = null;

    private DcMotor pickup = null;
    private DcMotor wheel = null;
    private Servo Claw = null;




    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, rotation;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    static int LAST_DIRECTION = logoFacingDirections.length - 1;
    static float TRIGGER_THRESHOLD = 0.2f;

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;






    boolean toggleB = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 3; // Left
        usbFacingDirectionPosition = 0; // Up

        updateOrientation();

        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;




        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        Arm = hardwareMap.dcMotor.get("Arm");
        Alien = (DcMotorEx) hardwareMap.dcMotor.get("Alien");
        Claw = hardwareMap.get(Servo.class, "Claw");
        wheel = hardwareMap.dcMotor.get("wheel");
        pickup = hardwareMap.dcMotor.get("pickup");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        Alien.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry

        double y = 0.75*-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y); // Remember, Y stick value is reversed
        double x = 0.5*gamepad1.left_stick_x * 1.1 * Math.abs(gamepad1.left_stick_x); // Counteract imperfect strafing
        double rx = (Math.abs(gamepad1.right_stick_x)/gamepad1.right_stick_x)*Math.min((gamepad1.right_stick_x * gamepad1.right_stick_x),0.5);
        if (gamepad1.right_stick_x == 0){rx = 0;}
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        double botHeading = orientation.getYaw(AngleUnit.RADIANS);



        if(gamepad1.x) {

            Alien.setVelocity(-1900);
        }
        else {

            if(gamepad1.b) {

               Alien.setVelocity(-3000);
            } else if (gamepad1.y) {
                Alien.setVelocity(-1500);
            } else if (gamepad1.a) {
                Alien.setVelocity(-1300);
            } else{
                if(gamepad1.dpad_up){
                    Alien.setVelocity(2600);
                }
                else {

                    Alien.setPower(0);
                }
            }


        }


        if(gamepad1.left_bumper) {
            Arm.setPower(1);
            wheel.setPower(1);
        }
        else {

            if(gamepad1.right_bumper) {
                Arm.setPower(1);
                wheel.setPower(-1);
            } else if (gamepad1.dpad_up) {
                Arm.setPower(-1);
                wheel.setPower(0);
            } else {
                Arm.setPower(0);
                wheel.setPower(0);
                pickup.setPower(0);
            }

        }



        telemetry.addData("Alien", Alien.getCurrentPosition());
        telemetry.addData("Arm", Arm.getCurrentPosition());
        telemetry.addData("Right Front", motorFrontRight.getCurrentPosition());
        telemetry.addData("Left Front", motorFrontLeft.getCurrentPosition());
        telemetry.addData("Right Back", motorBackRight.getCurrentPosition());
        telemetry.addData("Left Back", motorBackLeft.getCurrentPosition());

        telemetry.addData("A", toggleB);
        telemetry.addData("servo",Claw.getPosition());






        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Yaw (Z)", "%.2f Rad. (Heading)", orientation.getYaw(AngleUnit.DEGREES));





        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }



    @Override
    public void stop() {

    }
    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }

}