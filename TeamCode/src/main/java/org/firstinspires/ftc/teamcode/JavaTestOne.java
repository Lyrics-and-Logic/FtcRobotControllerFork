package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "JavaTestOne")
public class JavaTestOne extends LinearOpMode {

  private DcMotor LF;
  private DcMotor LR;
  private DcMotor RF;
  private DcMotor RR;
  private DcMotor ClawLift;
  private DcMotor ClawSlide;
  private CRServo ClawOpen;
  private DcMotor Carousel;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    LF = hardwareMap.get(DcMotor.class, "LF");
    LR = hardwareMap.get(DcMotor.class, "LR");
    RF = hardwareMap.get(DcMotor.class, "RF");
    RR = hardwareMap.get(DcMotor.class, "RR");
    ClawLift = hardwareMap.get(DcMotor.class, "Claw Lift");
    ClawSlide = hardwareMap.get(DcMotor.class, "Claw Slide");
    ClawOpen = hardwareMap.get(CRServo.class, "ClawOpen");
    Carousel = hardwareMap.get(DcMotor.class, "Carousel");

    // Put initialization blocks here.
    LF.setDirection(DcMotorSimple.Direction.REVERSE);
    LR.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        LF.setPower((((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) / (gamepad1.right_trigger + 1)) / 1.2);
        LR.setPower((((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) / (gamepad1.right_trigger + 1)) / 1.2);
        RF.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / (gamepad1.right_trigger + 1)) / 1.2);
        RR.setPower((((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) / (gamepad1.right_trigger + 1)) / 1.2);
        ClawLift.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        ClawSlide.setPower(gamepad2.left_stick_y);
        if (gamepad2.left_bumper) {
          ClawOpen.setPower(1);
        } else if (gamepad2.right_bumper) {
          ClawOpen.setPower(-1);
        } else {
          ClawOpen.setPower(0);
        }
        if (gamepad1.x) {
          Carousel.setPower(0.4);
        } else if (gamepad1.y) {
          Carousel.setPower(-0.4);
        } else {
          Carousel.setPower(0);
        }
        telemetry.update();
      }
    }
  }
}
