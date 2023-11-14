// 3 Rectangle Class:
package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ThreeRectanglesProcessor implements VisionProcessor {

  public Rect rectLeft = new Rect(110, 42, 40, 40);
  public Rect rectMiddle = new Rect(160, 42, 40, 40);
  public Rect rectRight = new Rect(210, 42, 40, 40);
  Selected selection = Selected.NONE;
  
  Mat submat = new Mat();
  Mat hsvMat = new Mat();
  
  @Override
  public void init(int width, int height, CameraCalibration calibration) {
  }
  
  @Override
  public Object processFrame(Mat frame, long captureTimeNanos) {
    Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
    
    double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
    double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
    double satRectRight = getAvgSaturation(hsvMat, rectRight);
    
    if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
      return Selected.LEFT;
    } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
      return Selected.MIDDLE;
    }
  return Selected.RIGHT;
  }
  
  protected double getAvgSaturation(Mat input, Rect rect) {
    submat = input.submat(rect);
    Scalar color = Core.mean(submat);
    return color.val[1];
  }
  
  private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
    int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
    int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
    int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
    int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
  
    return new android.graphics.Rect(left, top, right, bottom);
  }
  
  @Override
  public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    Paint selectedPaint = new Paint();
    selectedPaint.setColor(Color.RED);
    selectedPaint.setStyle(Paint.Style.STROKE);
    selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
    
    Paint nonSelectedPaint = new Paint(selectedPaint);
    nonSelectedPaint.setColor(Color.GREEN);
    
    android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
    android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
    android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);
    
    selection = (Selected) userContext;
    switch (selection) {
      case LEFT:
        canvas.drawRect(drawRectangleLeft, selectedPaint);
        canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
        canvas.drawRect(drawRectangleRight, nonSelectedPaint);
        break;
      case MIDDLE:
        canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
        canvas.drawRect(drawRectangleMiddle, selectedPaint);
        canvas.drawRect(drawRectangleRight, nonSelectedPaint);
        break;
      case RIGHT:
        canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
        canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
        canvas.drawRect(drawRectangleRight, selectedPaint);
        break;
      case NONE:
        canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
        canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
        canvas.drawRect(drawRectangleRight, nonSelectedPaint);
        break;
    }
  }
      
  public Selected getSelection() {
    return selection;
  }
  
  public enum Selected {
    NONE,
    LEFT,
    MIDDLE,
    RIGHT
  }
}


// Actual Opmode:
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.ThreeRectanglesProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous()
public class FirstVisionOpmode extends OpMode {
  private ThreeRectanglesProcessor visionProcessor;
  private VisionPortal visionPortal;
  
  @Override
  public void init() {
  visionProcessor = new FirstVisionProcessor();
  visionPortal =	VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "OpenCvInternalCamera.CameraDirection.BACK"), visionProcessor);
  }
  
  @Override
  public void init_loop() {
  }
  
  	@Override
  public void start() {
    visionPortal.stopStreaming();
    
    if (visionProcessor.getSelection() == LEFT) {
    	// do the left stuff, repeat for more, etc.
    }
  }

  @Override
  public void loop() {
    telemetry.addData("Identified", visionProcessor.getSelection());
  }
}





