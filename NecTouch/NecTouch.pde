import diewald_CV_kit.blobdetection.*; //<>// //<>// //<>// //<>//
import diewald_CV_kit.libraryinfo.*;
import diewald_CV_kit.utility.*;

import oscP5.*;
import netP5.*;
import java.util.*;
//import SimpleOpenNI.*;
//import Blobscanner.*;
//import hypermedia.video.*;
import java.awt.Rectangle;
import controlP5.*;

import KinectPV2.KJoint;
import KinectPV2.*;


ControlP5 controlP5;

Grabber gbTL;
Grabber gbTR;
Grabber gbBL;
Grabber gbBR;

Grabber[] grabbers;
Feedback feedback;
TUIOServer tuioServer;

TouchPoint simPoint;
boolean simActive, simAlreadyActive;


PVector leftHorizon, rightHorizon, upLPoint, upRPoint;

int gridLines = 4;
int upFactor = 80;

boolean showHelpers, showGrid, showInfos, showDrawingLines, showLabels, showFeedback, maskFloor;

boolean doCalibrate = false, doMask = false, mirrorMode = false;
boolean miniMode;

//SimpleOpenNI  context;
KinectPV2 context;

boolean enableRGB;

int[] planePixels, planeDepthPixels, planeDepthMap;

int imageWidth, imageHeight, pixelsLength, planePixelsLength;

PVector mainOffset;
boolean offsetting;
PVector tmpMouseOffset, tmpInitOffset;

boolean invertX, invertY, swapXY;

boolean nonLinearMode;

PGraphics planeMask;

int minDistance, minBlobSize, maxBlobSize;

PImage blobsImage;
//Detector bd;
ComputerVision cv;

int goodBlobsNumber, rawBlobsNumber;

TouchPoint[] touchPoints;

static int globalTouchPointIndex;

int minDiffTouch = 1, minDiff = 2, maxDiff = 3, 
  minDiffT = 2, maxDiffT = 10, 
  minBlobWeight, maxBlobWeight;


PImage blobsImageTouch;
PImage blobsImageBlank;

color[] colors = {
  color(255, 0, 0), color(0, 255, 0), color(20, 100, 255), color(255, 255, 50), color(255, 0, 255), color(0, 255, 255)
};

boolean criticalStop;

XML config;

boolean autoCalibrate;

boolean calibratePlane = false;

PVector topLeft = new PVector(imageWidth, imageHeight);
PVector bottomRight = new PVector(0, 0);
int[] floorMaskPixels = new int[pixelsLength];

void setup()
{
  println("setup");
  criticalStop = false;
  textMode(MODEL);


  config = loadXML("config.xml");

  if (config == null)
  {
    criticalStop = true;
    size(300, 100, P2D);
    background(0);
    println("config.xml does not exists in the data folder !"); 
    text("config.xml not found in the data folder !", 10, 10, 300, 20);
    return;
  }

  XML xmlWindow = config.getChild("window");
  mainOffset = new PVector(xmlWindow.getInt("offsetX", 0), xmlWindow.getInt("offsetY"), 0);
  int tw = miniMode?200:(xmlWindow.getInt("width", 640)+(int)mainOffset.x);
  int th = miniMode?40:(xmlWindow.getInt("height", 480)+(int)mainOffset.y);

  //TODO look minimode
  size(800, 600, P2D);

  frame.setSize(tw, th); 
  frame.setResizable(boolean(xmlWindow.getString("resizable", "true")));
  smooth();

  println("before CV");

  cv = new CVDiewald();
  cv.init(this, KinectPV2.WIDTHDepth, KinectPV2.HEIGHTDepth);
  println("after cv");
  //cv.allocate(imageWidth,imageHeight);


  context = new KinectPV2(this);
  //Enable point cloud
  context.enableDepthImg(true);
  context.enablePointCloud(true);

  context.init();

  println("AFTER");

  XML xmlKinect = config.getChild("kinect");

  frameRate(60);

  mirrorMode = boolean(xmlKinect.getString("mirror"));
  //context.setMirror(mirrorMode);


  XML xmlStartup = config.getChild("startup");
  //showHelpers = boolean(xmlStartup.getString("showHelpers", "true"));
  showGrid = boolean(xmlStartup.getString("showGrid", "true"));
  //showInfos = boolean(xmlStartup.getString("showInfos", "true"));
  //showDrawingLines = boolean(xmlStartup.getString("showDrawingLines", "true"));
  showFeedback = boolean(xmlStartup.getString("showFeedback", "true"));
  showLabels = boolean(xmlStartup.getString("showLabels", "true"));
  miniMode = boolean(xmlStartup.getString("miniMode", "true")); 
  autoCalibrate = boolean(xmlStartup.getString("autoCalibrate", "false"));
  println("AFTER xmls");

  XML child0 = config.getChild("grabberstl");
  gbTL = new Grabber(0, child0.getString("label", ""), child0.getInt("x", 
    50), child0.getInt("y", 50));

  XML childtr = config.getChild("grabberstr");
  gbTR = new Grabber(1, childtr.getString("label", ""), childtr.getInt(
    "x", 250), childtr.getInt("y", 50));

  XML childbr = config.getChild("grabbersbr");
  gbBR = new Grabber(2, childbr.getString("label", ""), childbr.getInt(
    "x", 250), childbr.getInt("y", 200));

  XML childbl = config.getChild("grabbersbl");
  gbBL = new Grabber(3, childbl.getString("label", ""), childbl.getInt(
    "x", 50), childbl.getInt("y", 200));

  grabbers = new Grabber[4];
  grabbers[0] = gbTL;
  grabbers[1] = gbTR;
  grabbers[2] = gbBR;
  grabbers[3] = gbBL;

  println("AFTER grabbers");
  maskFloor = false;

  simActive = false;
  simAlreadyActive = false;

  XML xmlFeedback = config.getChild("feedback");
  feedback = new Feedback(xmlFeedback.getInt("width", 100), xmlFeedback.getInt("height", 100));

  XML xmlTuio = config.getChild("tuio");
  tuioServer = new TUIOServer(xmlTuio.getString("host", "127.0.0.1"), xmlTuio.getInt("port", 3333));

  imageWidth = KinectPV2.WIDTHDepth;
  imageHeight = KinectPV2.HEIGHTDepth;
  pixelsLength = imageWidth * imageHeight;

  planeMask = createGraphics(imageWidth, imageHeight, P2D);

  blobsImage = createImage(imageWidth, imageHeight, ARGB);
  blobsImageTouch = createImage(imageWidth, imageHeight, ARGB);
  blobsImageBlank = createImage(imageWidth, imageHeight, ARGB);


  XML xmlDetection = config.getChild("detection");

  minDistance = xmlDetection.getInt("minDistance");

  minBlobSize = xmlDetection.getInt("minBlobSize");
  maxBlobSize = xmlDetection.getInt("maxBlobSize");
  nonLinearMode = boolean(xmlDetection.getString("nonLinear"));



  invertX =  boolean(xmlDetection.getString("invertX", "false"));
  invertY =  boolean(xmlDetection.getString("invertY", "false"));
  swapXY =  boolean(xmlDetection.getString("swapXY", "false"));

  touchPoints = new TouchPoint[0];


  controlP5 = new ControlP5(this);
  //WARNING TODO I don't know what do
  // controlP5.tab("miniMode");
  //controlP5.tab("default").activateEvent(true);
  // controlP5.tab("miniMode").activateEvent(true);

  //controlP5.addToggle("showHelpers",showGrid,10,10,20,20);
  //controlP5.addToggle("showDrawingLines",showGrid,10,10,20,20);

  //controlP5.addToggle("showGrid", showGrid, 10, 30, 10, 10).captionLabel().style().margin(-12, 0, 0, 15);
  //controlP5.addToggle("showFeedback", showFeedback, 10, 45, 10, 10).captionLabel().style().margin(-12, 0, 0, 15);
  //controlP5.addToggle("showLabels", showLabels, 10, 60, 10, 10).captionLabel().style().margin(-12, 0, 0, 15);
  //END WARNING TODO I don't know what do

  RadioButton r = controlP5.addRadio("enableRGB", 100, 30);
  r.deactivateAll(); // use deactiveAll to not make the first radio button active.

  r.add("RGB", 1);
  r.add("Depth", 0);

  enableRGB = boolean(xmlKinect.getString("enableRGB"));
  if (enableRGB) 
  {
    r.activate("RGB");
  } else
  {
    r.activate("Depth");
  }

  /* controlP5.addBang("calibratePlane", 100, 70, 20, 20).captionLabel().style().margin(-17, 0, 0, 25);
   
   controlP5.addToggle("mirrorMode", mirrorMode, 220, 10, 10, 10).captionLabel().style().margin(-12, 0, 0, 15);
   controlP5.addToggle("doMask", doMask, 220, 25, 10, 10).captionLabel().style().margin(-12, 0, 0, 15);
   controlP5.addToggle("invertX", invertX, 220, 40, 10, 10).captionLabel().style().margin(-12, 0, 0, 15);
   controlP5.addToggle("invertY", invertY, 220, 55, 10, 10).captionLabel().style().margin(-12, 0, 0, 15);
   controlP5.addToggle("swapXY", swapXY, 220, 70, 10, 10).captionLabel().style().margin(-12, 0, 0, 15);
   
   controlP5.addNumberbox("minDistance", minDistance, 330, 10, 50, 14).captionLabel().style().margin(-12, 0, 0, 62);
   controlP5.addNumberbox("maxDistance", maxDistance, 330, 30, 50, 14).captionLabel().style().margin(-12, 0, 0, 62);
   controlP5.addNumberbox("minBlobSize", minBlobSize, 330, 50, 50, 14).captionLabel().style().margin(-12, 0, 0, 62);
   controlP5.addNumberbox("maxBlobSize", maxBlobSize, 330, 70, 50, 14).captionLabel().style().margin(-12, 0, 0, 62);
   */
  Slider s1 = controlP5.addSlider("gridLines", 0, 20, 330, 100, 80, 10);
  s1.setNumberOfTickMarks(20);

  topLeft = new PVector(imageWidth, imageHeight);
  bottomRight = new PVector(0, 0);
}

int[] getDepthMap() {
  PImage depth = context.getPointCloudDepthImage();
  depth.loadPixels();
  int[] depthMap = depth.pixels;
  return depthMap;
}

void draw() {
  if (criticalStop)
    return;

  background(0);
  // context.update();
  // draw
  pushMatrix();
  translate(mainOffset.x, mainOffset.y);

  PImage kinectImage = null;
  int i;
  int[] depthMap = getDepthMap();

  if (autoCalibrate && planePixels == null) {
    println("AutoCalibrate !");
    calibratePlane();
  }

  if (doCalibrate) {
    println("calibration done !");
    calibratePlane();
  }

  if (!miniMode) {
    if (enableRGB) {
      kinectImage = context.getPointCloudDepthImage();
    } else {
      kinectImage = context.getPointCloudDepthImage();
    }

    if (doMask && planePixels != null) {
      kinectImage.mask(planePixels);
    }

    image(kinectImage, 0, 0);
  }

  simulationAndTransformation();

  if (planePixelsLength > 0) {

    processImageBlob(kinectImage, depthMap);
    processBlobs();

    if (!miniMode) {
      if (maskFloor) {
        kinectImage.mask(floorMaskPixels);
        image(kinectImage, 0, 0);
      }

      image(blobsImage, 0, 0);
      image(blobsImageTouch, 0, 0);
    }
  }

  popMatrix(); // mainOffset pop

  if (showFeedback && !miniMode) {
    feedback.draw();

    for (i = 0; i < touchPoints.length; i++) {

      int c = getColorForIndex(touchPoints[i].id);
      feedback.drawPoint(touchPoints[i], touchPoints[i].id, c);
      pushMatrix();
      translate(mainOffset.x, mainOffset.y);
      touchPoints[i].drawPointReel(c);
      popMatrix();
    }
  }

  if (!miniMode) {
    if (showInfos) {
      drawGUI();
    }
    text("'i' - Show / Hide infos", 10, 10, 200, 20);
  }

  pushStyle();
  textAlign(RIGHT);
  noStroke();
  fill(0, 160);
  rect(width - 200, height - 40, 100, 40);
  fill(255);
  text("Framerate " + (int) frameRate, width - 100, height - 35, 90, 15);
  // text("Raw blobs "+rawBlobsNumber, width-100, height-35, 90, 15);
  text("Active blobs " + goodBlobsNumber, width - 100, height - 15, 90, 
    15);
  popStyle();

  if (calibratePlane) {
    calibratePlane = false;
    calibratePlane();
  }
  //    
  //    ellipse(mouseX,mouseY,400,400);
  //    image(blobsImage,mouseX,mouseY);
}

private void drawGUI() {
  fill(0, 160);
  noStroke();
  rect(0, 0, 300, 310);

  fill(255);
  pushStyle();
  if (showGrid)
    fill(100, 200, 20);
  text("g : Show / Hide Grid", 10, 30, 200, 20);
  popStyle();

  text("8 / 2 : Increase / Decrease grid densityr", 10, 50, 250, 20);

  pushStyle();
  if (showDrawingLines)
    fill(100, 200, 20);
  text("d : Show / Hide Drawing Lines", 10, 70, 200, 20);
  popStyle();

  pushStyle();
  if (showHelpers)
    fill(100, 200, 20);
  text("h : Show / Hide Helpers", 10, 90, 200, 20);
  popStyle();

  pushStyle();
  if (enableRGB)
    fill(100, 200, 20);
  text("k : Switch RGB / Depth Image mode", 10, 110, 200, 20);
  popStyle();

  pushStyle();
  if (mirrorMode)
    fill(100, 200, 20);
  text("r : Toggle Kinect Mirror Mode", 10, 130, 200, 20);
  popStyle();

  pushStyle();
  if (showLabels)
    fill(100, 200, 20);
  text("l : Show / Hide Labels", 10, 150, 200, 20);
  popStyle();

  pushStyle();
  if (showFeedback)
    fill(100, 200, 20);
  text("f : Show / Hide Feedback", 10, 170, 200, 20);
  popStyle();

  text("c : Calibrate plane", 10, 190, 200, 20);

  pushStyle();
  if (doMask)
    fill(100, 200, 20);
  text("m : Toggle plane mask mode", 10, 210, 250, 20);
  popStyle();

  pushStyle();
  if (swapXY)
    fill(100, 200, 20);
  text("w : Toggle swapXY", 10, 230, 250, 20);
  popStyle();

  pushStyle();
  if (invertX)
    fill(100, 200, 20);
  text("x : Toggle invertX", 10, 250, 250, 20);
  popStyle();

  pushStyle();
  if (invertY)
    fill(100, 200, 20);
  text("y : Toggle invertY", 10, 270, 250, 20);
  popStyle();

  text("s : Save settings", 10, 290, 250, 20);

  pushStyle();
  fill(0, 160);
  rect(0, height - 40, width, 40);

  if (!miniMode) {
    fill(255);
    text("Min (n/N) : " + minDiffTouch, 
      10, height - 75, 500, 20);

    minBlobWeight = 35;
    maxBlobWeight = 1210;

    text("Min / Max diff (v/V - b/B) : " + minDiff
      + " -> " + maxDiff, 10, height - 35, 400, 20);
    text("Min / Max diff Touch (','/';' - '.'/':') : " + minDiffT
      + " -> " + maxDiffT, 10, height - 55, 400, 20);
    text("Min / Max blob size ((Alt or Nothing) & '+' / '-') : "
      + minBlobWeight + " -> " + maxBlobWeight, 10, height - 15, 
      450, 20);
  }
  popStyle();
}



private void simulationAndTransformation() {
  int i;
  leftHorizon = lineIntersection(gbTL.x, gbTL.y, gbBL.x, gbBL.y, gbTR.x, 
    gbTR.y, gbBR.x, gbBR.y);
  rightHorizon = lineIntersection(gbTL.x, gbTL.y, gbTR.x, gbTR.y, gbBL.x, 
    gbBL.y, gbBR.x, gbBR.y);

  PVector upPoint = new PVector(gbBL.x, gbBL.y - upFactor);

  if (rightHorizon != null && leftHorizon != null) {

    upLPoint = lineIntersection(gbTL.x, gbTL.y, gbTL.x, gbTL.y - 1, 
      leftHorizon.x, leftHorizon.y, upPoint.x, upPoint.y);
    upRPoint = lineIntersection(gbBR.x, gbBR.y, gbBR.x, gbBR.y - 1, 
      rightHorizon.x, rightHorizon.y, upPoint.x, upPoint.y);

    if (!miniMode) {

      if (showDrawingLines && !nonLinearMode) {
        ellipse(leftHorizon.x, leftHorizon.y, 10, 10);
        ellipse(rightHorizon.x, rightHorizon.y, 10, 10);

        stroke(200, 150);
        line(leftHorizon.x, leftHorizon.y, gbBL.x, gbBL.y);
        line(rightHorizon.x, rightHorizon.y, gbTL.x, gbTL.y);
        line(leftHorizon.x, leftHorizon.y, gbBR.x, gbBR.y);
        line(rightHorizon.x, rightHorizon.y, gbBR.x, gbBR.y);

        line(leftHorizon.x, leftHorizon.y, rightHorizon.x, 
          rightHorizon.y);
        line(leftHorizon.x, leftHorizon.y, rightHorizon.x, 
          rightHorizon.y);
      }

      if (showHelpers && !nonLinearMode) {
        drawRepere(leftHorizon, gbBL, gbTL, upPoint, upLPoint);
        drawRepere(rightHorizon, gbBL, gbBR, upPoint, upRPoint);
      }

      if (showGrid) {
        drawGrid(leftHorizon, rightHorizon, gbBL, gbTL, gbTR, gbBR);
        drawGrid(rightHorizon, leftHorizon, gbBL, gbBR, gbTL, gbTR);
      }
    }

    if (!miniMode) {
      for (i = 0; i < 4; i++) {
        grabbers[i].draw();
        stroke(255);
        line(grabbers[i].x, grabbers[i].y, grabbers[(i + 1) % 4].x, 
          grabbers[(i + 1) % 4].y);
      }
    }

    if (simActive) {

      PVector targetPoint = getProjectedPoint(new PVector(mouseX
        - mainOffset.x, mouseY - mainOffset.y));

      // force same point at first time to avoid non-sense with speed
      // and acc computed values
      if (!simAlreadyActive) {
        simPoint = new TouchPoint(targetPoint.x, targetPoint.y, 10, 
          null, false, null);
        simPoint.setState("new");
      } else {
        simPoint.lastPoint = new TouchPoint(simPoint.x, simPoint.y, 
          10, null, true, null);
        simPoint.x = targetPoint.x;
        simPoint.y = targetPoint.y;
        simPoint.setState("update");
      }

      if (showFeedback) {
        pushMatrix();
        translate(-mainOffset.x, -mainOffset.y);
        feedback.drawPoint(targetPoint, -1, color(255, 255, 255));
        popMatrix();
      }
    } else {
      if (simAlreadyActive) {
        // Point destroy
        simAlreadyActive = false;
        simPoint.setState("destroy");
        // tuioServer.send("destroy", simPoint);
      }
    }
  }
}



public void calibratePlane() {
  int[] depthMap = getDepthMap();
  int i;

  planeMask.colorMode(ALPHA);
  planeMask.beginDraw();
  planeMask.background(100);
  planeMask.fill(255);
  planeMask.noStroke();
  planeMask.beginShape();
  for (i = 0; i < grabbers.length; i++) {
    planeMask.vertex(grabbers[i].x, grabbers[i].y);
  }
  planeMask.endShape(CLOSE);
  planeMask.endDraw();

  PImage tempMask = planeMask.get();

  /*
     * PImage planeMask = new PImage(imageWidth,imageHeight,ALPhA);
   * planeMask.loadPixels();
   */
  tempMask.loadPixels();
  planePixels = tempMask.pixels;
  /* planeMask.updatePixels(); */

  boolean[] planeBuffer = new boolean[pixelsLength];

  planePixelsLength = 0;

  // Get the length of the plane pixels array
  for (i = 0; i < pixelsLength; i++) {
    if (planePixels[i] == -1) {
      planeBuffer[i] = true;
      planePixelsLength++;
    }
  }

  // fill the planeDepthPixels array with pixels indexes and planeDepthMap
  // with depthMap data

  planeDepthPixels = new int[planePixelsLength];
  planeDepthMap = new int[planePixelsLength];

  int planeDepthPixelsIndex = 0;
  for (i = 0; i < pixelsLength; i++) {
    if (planeBuffer[i]) {
      planeDepthPixels[planeDepthPixelsIndex] = i;
      planeDepthMap[planeDepthPixelsIndex] = depthMap[i];
      planeDepthPixelsIndex++;
    }
  }
}




PVector getProjectedPoint(PVector touchPoint)
{


  pushStyle();
  noStroke();
  pushMatrix();
  translate(touchPoint.x, touchPoint.y);
  noFill();
  stroke(200, 150);
  ellipse(0, 0, 10, 10);
  popMatrix();
  popStyle();



  float targetX, targetY;
  if (!nonLinearMode)
  {
    targetX = calculatePerspCoord(leftHorizon, rightHorizon, touchPoint, gbBL, gbBR, upRPoint, gbTL, gbTR);
    targetY = 1 - calculatePerspCoord(rightHorizon, leftHorizon, touchPoint, gbBL, gbTL, upLPoint, gbBR, gbTR);
  } else
  {
    targetX = calculateNonLinearCoord(gbTL, gbTR, gbBL, gbBR, touchPoint);
    targetY = calculateNonLinearCoord(gbTR, gbBR, gbTL, gbBL, touchPoint);
  }

  targetX = constrain(targetX, 0, 1);
  targetY = constrain(targetY, 0, 1);

  if (swapXY)
  {
    float tmp = targetX;
    targetX = targetY;
    targetY = tmp;
  }

  if (invertX) targetX = 1-targetX;
  if (invertY) targetY = 1-targetY;

  //println("targetX / Y ="+targetX+", "+targetY);

  return new PVector(targetX, targetY);
}

color getColorForIndex(int i)
{
  return colors[i%colors.length];
}


void setMiniMode()
{
  if (miniMode)
  {
    size(200, 40);
    frame.setSize(200, 80);
  } else
  {
    size(imageWidth+(int)mainOffset.x, imageHeight+(int)mainOffset.y);
    frame.setSize(width+80, height+80);
  }
}