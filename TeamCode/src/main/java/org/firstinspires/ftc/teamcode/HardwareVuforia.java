package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class HardwareVuforia {
    private double [][] VuforiaCoords = { {0, 0, 0},
                                         {0, 0, 0} } ;
    public OpenGLMatrix lastLocation = null;

    private VuforiaTrackables relicTrackables = null;
    private VuforiaTrackable relicTemplate = null;
    public RelicRecoveryVuMark vuMark = null;

    public enum vuForiaCoord {
        tX,
        tY,
        tZ,
        rX,
        rY,
        rZ
    }
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {


        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "Af9ksOb/////AAAAGeC5pTEf7UNjuhNNYISpFKpt8KPEneyWRu3u6EqzGuTPAuvL51N5ilizvCYpiViD5i2WeURlXcPZ6PcAmOelb9gJrNXs6SaMhCfuoLdWhQ63QnoTv3RWAest5W1vfgnhn7JQz2R3itmAu9fhSipQEBzeLIguiXDDR0E3ne22sb8F1YMj3WTN+htbOMNtRTWfV3PUrHKAjA/rdBKbG1vEOGiyltjxNVBK0cwIS5wvA6T8wFIfbdK5xcIKE78vZRgNptVTqs99R12P0N/Si0Pd9VdHVyhjcIKA5gVE7seUsa2trMl2hg7ZHV91m3WOwvmJamGDq3bbxlm+544v24bY7Erya5mhAGI2HL1fxJlmDW0z";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the Front (LowRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

    }
        //telemetry.addData(">", "Press Play to start");
        //telemetry.update();
        //waitForStart();
    public void start () {
        relicTrackables.activate();
    }
    public RelicRecoveryVuMark GetLocation () {

        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */

        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return (vuMark) ;
    }

  /*  public double GetX () {


        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return (0.0);
    }

    public double GetY () {


        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return (0.0);
    }
*/

  public void updateVuforiaCoords () {

      OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
      if (pose != null) {
          VectorF trans = pose.getTranslation();
          Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

          // Extract the X, Y, and Z components of the offset of the target relative to the robot
          double tX = trans.get(0);
          double tY = trans.get(1);
          double tZ = trans.get(2);

          // Extract the rotational components of the target relative to the robot
          double rX = rot.firstAngle;
          double rY = rot.secondAngle;
          double rZ = rot.thirdAngle;

          VuforiaCoords [0] [0] = tX ;
          VuforiaCoords [0] [1] = tY ;
          VuforiaCoords [0] [2] = tZ ;

          VuforiaCoords [1] [0] = rX ;
          VuforiaCoords [1] [1] = rY ;
          VuforiaCoords [1] [2] = rZ ;
      }
  }

  public double getVuforiaCoords (vuForiaCoord axis) {
      double outPut = 0 ;

      if (axis == vuForiaCoord.tX) {outPut = VuforiaCoords [0][0] ;}
      if (axis == vuForiaCoord.tY) {outPut = VuforiaCoords [0][1] ;}
      if (axis == vuForiaCoord.tZ) {outPut = VuforiaCoords [0][2] ;}

      if (axis == vuForiaCoord.rX) {outPut = VuforiaCoords [1][0] ;}
      if (axis == vuForiaCoord.rY) {outPut = VuforiaCoords [1][1] ;}
      if (axis == vuForiaCoord.rZ) {outPut = VuforiaCoords [1][2] ;}

      return  outPut ;
  }






            //if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */


                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */

              //  OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                //telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */

                //if (pose != null) {
                  //  VectorF trans = pose.getTranslation();
                    //Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                /*    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }

    */

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    public void addTelemetry(Telemetry telemetry) {
        vuMark = GetLocation();
        telemetry.addLine()
             .addData("VuMark", "%s visible", GetLocation());
        telemetry.addLine()
                .addData("X Vuforia ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(getVuforiaCoords(vuForiaCoord.tZ));
                    }
                })
                .addData("X Rot Vuforia ", new Func<String>() {
                    @Override
                    public String value() {
                        return FormatHelper.formatDouble(getVuforiaCoords(vuForiaCoord.rZ));
                    }
                });
    }
}
