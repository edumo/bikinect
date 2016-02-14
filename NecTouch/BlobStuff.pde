
private void processImageBlob(PImage kinectImage, int[] depthMap) {
  int i;

  blobsImage.set(0, 0, blobsImageBlank);
  blobsImageTouch.set(0, 0, blobsImageBlank);

  blobsImage.loadPixels();
  blobsImageTouch.loadPixels();

  floorMaskPixels = new int[pixelsLength];

  for (i = 0; i < planePixelsLength; i++) {
    int targetPixelIndex = planeDepthPixels[i];
    int refDepth = (int) red(planeDepthMap[i]);
    int curDepth = (int) red(depthMap[targetPixelIndex]);
    // filterImage.pixels[targetPixelIndex] = color(255,0,0,50);
    int diffDepth = refDepth - curDepth;

    // blobsImage.pixels[targetPixelIndex] = color(0,225,120,150);

    if (diffDepth > minDiffTouch) { 
      // ESTAMOS POR ENCIMA DEL �REA DE CONTACTO

      if (minDiff < diffDepth && diffDepth < maxDiffT) {
        // ESTAMOS EN ZONA DE TOUCH
        blobsImageTouch.pixels[targetPixelIndex] = color(0, 255, 0, 
          150);
      }
      // QUiZA ESTAMOS EN ZONA DE BLOB
      if (maskFloor) {
        floorMaskPixels[targetPixelIndex] = 255;
      }

      // blobsImage.pixels[targetPixelIndex] =
      // color(0,50,255,150);

      if (diffDepth < maxDiff) {
        // AHORA S� ESTAMOS EN ZONA DE BLOB
        int reelX = targetPixelIndex % imageWidth;
        int reelY = floor(targetPixelIndex / imageWidth);

        topLeft.x = min(topLeft.x, reelX);
        topLeft.y = min(topLeft.y, reelY);
        bottomRight.x = max(bottomRight.x, reelX);
        bottomRight.y = max(bottomRight.y, reelY);

        // println(curDepth+"-"+refDepth+" = "+(curDepth-refDepth));
        blobsImage.pixels[targetPixelIndex] = color(255, 0, 0, 150);
      }
    }
  }

  blobsImage.updatePixels();
}

public void processBlobs() {
  if (topLeft.x < bottomRight.x) {

    topLeft.x -= 10;
    topLeft.y -= 10;
    bottomRight.x += 10;
    bottomRight.y += 10;

    int rectW = (int) (bottomRight.x - topLeft.x);
    int rectH = (int) (bottomRight.y - topLeft.y);

    if (miniMode) {
      pushStyle();
      stroke(40, 120, 230, 100);
      strokeWeight(1);
      noFill();
      rect(topLeft.x, topLeft.y, rectW, rectH);
      popStyle();
    }

    // blobsImage = blobsImage.get((int)topLeft.x, (int)topLeft.y,
    // rectW, rectH);*/
    processBlobs2(blobsImage, blobsImageTouch, topLeft, rectW, rectH);
  } else {
    processBlobs2(blobsImage, blobsImageTouch, null, 0, 0);
  }
}


public void processBlobs2(PImage img, PImage touchImg, PVector offset, 
  int w, int h) {

  goodBlobsNumber = 0;
  // goodBlobsNumberConvexityDefect = 0;
  rawBlobsNumber = 0;

  TouchPoint[] blobPoints;


  if (w == 0 || h == 0) {
    blobPoints = new TouchPoint[0];
  } else {

    List<TouchPoint> pointList = new ArrayList<NecTouch.TouchPoint>();

    BiBlob[] blobs = new BiBlob[0];

    blobs = cv.findBlobs(blobsImage);

    processBlobsTouchs(pointList, blobs);

    text("" + goodBlobsNumber, 100, 100);

    blobPoints = new TouchPoint[pointList.size()];
    blobPoints = pointList.toArray(blobPoints);
  }
  // linkBlobs(blobPoints, touchPoints, goodBlobsNumber);

  touchPoints = blobPoints;
  List<Object> msgList = new ArrayList<Object>();
  for (int hh = 0; hh < touchPoints.length; hh++) {
    TouchPoint point = touchPoints[hh];
    msgList.add("" + point.x);
    msgList.add("" + point.y);
  }

  Object[] msg = new Object[msgList.size()];
  msgList.toArray(msg);

  // if (!simAlreadyActive) {
  // // New point
  // tuioServer.send("update", simPoint);
  // simAlreadyActive = true;
  // } else {
  // // Already there
  // tuioServer.send("update", simPoint);
  // }
}

public TouchPoint buildTouchPointFromBlob(PVector reelCoordVec, 
  PVector tmpVec, BiBlob blob) {

  // List<PVector> vertxList = new ArrayList<PVector>();
  //
  PVector[] vertexPvector = new PVector[blob.getPoints().length];
  for (int j = 0; j < blob.getPoints().length; j += 1) {
    PVector temp = getProjectedPoint(new PVector(blob.getPoints()[j].x, 
      blob.getPoints()[j].y));
    vertexPvector[j] = new PVector(temp.x, temp.y);
  }

  TouchPoint point = new TouchPoint(tmpVec.x, tmpVec.y, blob.getArea(), 
    reelCoordVec, false, vertexPvector);

  return point;
}
private void processBlobsTouchs(List<TouchPoint> pointList, BiBlob[] blobs) {
  int i;
  // rawBlobsNumber = blobs.length;
  // blobPoints = new TouchPoint[rawBlobsNumber];
  for (i = 0; i < blobs.length; i++) {
    // Rectangle r = blobs[i].rectangle;

    BiBlob blob = blobs[i];

    PVector reelCoordVec = new PVector(blob.getCentroid().x, 
      blob.getCentroid().y);
    PVector tmpVec = getProjectedPoint(reelCoordVec);

    TouchPoint point = buildTouchPointFromBlob(reelCoordVec, tmpVec, 
      blob);
    point.touch = true;
    pointList.add(point);
    goodBlobsNumber++;

    if (!miniMode) {
      text("x:" + blob.getCentroid().x, blob.getCentroid().x + 10, 
        blob.getCentroid().y);
      text("y:" + blob.getCentroid().y, blob.getCentroid().x + 10, 
        blob.getCentroid().y + 20);
    }
  }
}


void processBlobs(PImage img, PVector offset, int w, int h)
{
  goodBlobsNumber = 0;
  rawBlobsNumber = 0;
  TouchPoint[] blobPoints;
  int i;

  if (w == 0 || h == 0)
  {
    blobPoints = new TouchPoint[0];
  } else
  {

    /*bd = new Detector(this,0,0, blobsImage.width, blobsImage.height, 255 );
     bd.findBlobs(blobsImage.pixels, blobsImage.width, blobsImage.height);
     bd.loadBlobsFeatures();// to call always before to use a method returning or processing a blob feature
     bd.weightBlobs(true);
     
     rawBlobsNumber = bd.getBlobsNumber();*/

    /*cv.copy(blobsImage);
     cv.ROI((int)offset.x, (int)offset.y, w, h);
     
     Blob[] blobs = cv.blobs(minBlobSize, maxBlobSize, 20, false, 4);
     rawBlobsNumber = blobs.length;*/

    BiBlob[] blobs = new BiBlob[0];// cv.blobs(minBlobWeight, 50000, 20,
    // false, 25, false); // blobs
    // javacvPro
    blobs = cv.findBlobs(blobsImage);

    blobPoints = new TouchPoint[rawBlobsNumber];

    for (i = 0; i < rawBlobsNumber; i++)
    {
      //Rectangle r = blobs[i].rectangle;

      PVector reelCoordVec = blobs[i].getCentroid();
      PVector tmpVec = getProjectedPoint(reelCoordVec);
      //  blobPoints[goodBlobsNumber] = new TouchPoint(tmpVec.x, tmpVec.y, blobs[i].getArea(), reelCoordVec, false);
      //println(reelCoordVec+" /" +tmpVec);
      goodBlobsNumber++;
    }

    while (blobPoints.length > goodBlobsNumber) blobPoints = (TouchPoint[]) shorten(blobPoints);
  }


  int pLen = touchPoints.length;

  float minDist = 0;
  int minIndex = -1;
  float curDist = 0;

  if (goodBlobsNumber >= pLen)
  {

    //println("");
    //println("*** more or equal");
    for (i = 0; i<pLen; i++)
    {
      //println("touchPoint " + i +", id :"+touchPoints[i].id);

      minIndex = -1;
      curDist = 0;
      minDist = 0;

      for (int j=0; j<goodBlobsNumber; j++)
      {
        if (blobPoints[j].linked) {
          //println(" -> blob "+j+" is already linked");
          //println(" -> test distance with blob "+j+" :"+curDist);
        } else {
          curDist = PVector.dist(touchPoints[i], blobPoints[j]);
          //println(" -> test distance with blob "+j+" :"+curDist);

          if (minIndex == -1 || curDist < minDist)
          {
            minDist = curDist;
            minIndex = j;
          }
        }
      }

      //println(" -> linked with index :"+minIndex+", distance "+round(minDist)+", tmpId = "+blobPoints[minIndex].id+", touchId = "+touchPoints[i].id);
      blobPoints[minIndex].id = touchPoints[i].id;
      blobPoints[minIndex].lastPoint = touchPoints[i];
      blobPoints[minIndex].linked = true;

      /*if(PVector.dist(blobPoints[minIndex], blobPoints[minIndex].lastPoint) < .005)
       {
       blobPoints[minIndex] = blobPoints[minIndex].lastPoint;
       }*/

      //blobPoints[minIndex].x = blobPoints[minIndex].lastPoint.x + (blobPoints[minIndex].x - blobPoints[minIndex].lastPoint.x) * .5;
      //blobPoints[minIndex].y = blobPoints[minIndex].lastPoint.y + (blobPoints[minIndex].y - blobPoints[minIndex].lastPoint.y) * .5;
    }



    for (i = 0; i< goodBlobsNumber; i++)
    {

      if (!blobPoints[i].linked)
      {
        //New point
        //println("new Point");
        blobPoints[i].setState("new");
      } else
      {
        //blobPoints[i].setState("update");
      }
    }
  } else
  {
    //println("************************ LESS ***");

    for (i = 0; i<pLen; i++)
    {
      touchPoints[i].linked = false;
    }


    for (i = 0; i<goodBlobsNumber; i++)
    {
      //println("blobPoint" + i +", id :"+blobPoints[i].id);

      minIndex = -1;
      curDist = 0;
      minDist = 0;

      TouchPoint[] alivePoints = new TouchPoint[0];

      for (int j=0; j<pLen; j++)
      {
        if (touchPoints[j].linked) {
          // println(" -> touchpoint "+j+" is already linked");
        } else {

          curDist = PVector.dist(blobPoints[i], touchPoints[j]);
          //println(" -> test distance with touchpoint "+j+" :"+curDist);

          if (minIndex == -1 || curDist < minDist)
          {
            minDist = curDist;
            minIndex = j;
          }
        }
      }

      if (minIndex != -1)
      {
        //println(" -> linked with index :"+minIndex+", distance "+minDist+", touchId :"+touchPoints[minIndex].id);
        blobPoints[i].id = touchPoints[minIndex].id;
        blobPoints[i].lastPoint = touchPoints[minIndex];
        touchPoints[minIndex].linked = true;
      }
    }
  }


  touchPoints = blobPoints;
  tuioServer.send("update", touchPoints);
}