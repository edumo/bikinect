

void saveConfig()
{
  for (int i=0; i<grabbers.length; i++)
  {
    config.getChild("grabbers").getChild(i).setInt("x", (int)grabbers[i].x);
    config.getChild("grabbers").getChild(i).setInt("y", (int)grabbers[i].y);
  }



  config.getChild("detection").setInt("minDistance", minDistance);

  config.getChild("detection").setInt("minBlobSize", minBlobSize);
  config.getChild("detection").setInt("maxBlobSize", maxBlobSize);

  config.getChild("startup").setString("miniMode", str(miniMode));
  config.getChild("startup").setString("showHelpers", str(showHelpers));
  config.getChild("startup").setString("showGrid", str(showGrid));
  config.getChild("startup").setString("showInfos", str(showInfos));
  config.getChild("startup").setString("showDrawingLines", str(showDrawingLines));
  config.getChild("startup").setString("showFeedback", str(showFeedback));
  config.getChild("startup").setString("showLabels", str(showLabels));
  config.getChild("startup").setString("autoCalibrate", str(autoCalibrate));

  config.getChild("kinect").setString("mirror", str(mirrorMode));
  config.getChild("kinect").setString("enableRGB", str(enableRGB));

  config.getChild("detection").setString("invertX", str(invertX));
  config.getChild("detection").setString("invertY", str(invertY));
  config.getChild("detection").setString("swapXY", str(swapXY));

  println(config.toString());
  //config.save("data/config.xml");
  println("config saved !");
}

void mousePressed()
{
  if (criticalStop) return;

  Boolean grabberPressed = false;
  for (int i=0; i<4; i++)
  {
    if (grabbers[i].mousePressed())
    {
      grabberPressed = true;
      return;
    }
  }

  if (pixelInPoly(grabbers, new PVector(mouseX-mainOffset.x, mouseY-mainOffset.y)))
  {
    simActive = true;
    simAlreadyActive = false;
  } else if (mouseY > 110)
  {
    println("mouseY :"+mouseY);
    tmpMouseOffset = new PVector(mouseX, mouseY);
    tmpInitOffset = mainOffset;
    offsetting = true;
  }
}

void mouseReleased()
{
  if (criticalStop) return;

  for (int i=0; i<4; i++)
  {
    grabbers[i].pressed = false;
  }

  simActive = false;
  offsetting = false;
}


void controlEvent(ControlEvent e)
{
  if (e.isTab())
  {
    //TODO
    //miniMode = e.tab().name().equals("miniMode");
    //setMiniMode();
  } else {
    println(e.isGroup());
    if (e.isGroup())
    {
      println("group :"+ e.getName()+"/"+e.getValue());
      enableRGB = e.getValue() == 1?true:false;
    } else
    {
      String n = e.getName(); 
      if (n.equals("mirrorMode"))
      {
        //TODO
        //context.setMirror(mirrorMode);
      }
    }
  }
}


void keyPressed(KeyEvent e)
{

  if (criticalStop) return;

  switch(key)
  {

  case 'h':
    showHelpers = !showHelpers;
    break;

  case 'g':
    showGrid = !showGrid;
    break;

  case 'd':
    showDrawingLines  = !showDrawingLines;
    break;

  case 'i':
    showInfos = !showInfos;
    break;

  case '8':
    gridLines++;
    break;

  case '2':
    gridLines--;
    break;

  case 'l':
    showLabels = !showLabels;
    break;

  case 'f':
    showFeedback= !showFeedback;
    break;

  case 'p':
    doCalibrate = !doCalibrate;
    break;

  case 'c':

    calibratePlane = true;
    break;

  case 'k':
    enableRGB = !enableRGB;
    break;

  case 'r':
    mirrorMode = !mirrorMode;
    //TODO
    //context.setMirror(mirrorMode);
    break;

  case 'w':
    swapXY = !swapXY;
    break;

  case 'x':
    invertX = !invertX;
    break;

  case 'y':
    invertY = !invertY;
    break;

  case 's':
    saveConfig();
    break;
  case 'b':
    maxDiff--;
    break;
  case 'B':
    maxDiff++;
    break;
  case 'n':
    minDiffTouch--;
    break;
  case 'N':
    minDiffTouch++;
    break;
  case 'v':
    minDiff--;
    break;
  case 'V':
    minDiff++; 
    break;
  case ',':
    minDiffT--;
    break;
  case ';':
    minDiffT++; 
    break;
  case '.':
    maxDiffT--;
    break;
  case ':':
    maxDiffT++; 
    break;

  case 'm':

    doMask = !doMask;
    maskFloor = doMask;
    println("doMask & maskFloor "+doMask);
    if (doMask && planePixels == null)
    {
      calibratePlane();
    }

    break;

  case ' ':
    miniMode = !miniMode;
    controlP5.getTab("miniMode").setActive(miniMode);
    controlP5.getTab("default").setActive(!miniMode);
    setMiniMode();
  }

  println(e.getKeyCode());
  switch(e.getKeyCode())
  {

  case 107:
  case 47:
    if (e.isShiftDown())
    {
      //  maxDistance++;
    } else if (e.isControlDown())
    {
      minDistance++;
    } else if (e.isAltDown())
    {
      maxBlobSize++;
    } else
    {
      minBlobSize++;
    }
    break;

  case 109:
  case 61:
    if (e.isShiftDown())
    {
      //  maxDistance--;
    } else if (e.isControlDown())
    {
      minDistance--;
    } else if (e.isAltDown())
    {
      if (maxBlobSize > 0) maxBlobSize-- ;
    } else
    {
      if (minBlobSize > 0) minBlobSize--;
    }
    break;


  case 37:
    mainOffset.x -=2;
    break;
  case 38:
    mainOffset.y -=2;
    break;
  case 39:
    mainOffset.x +=2;
    break;
  case 40:
    mainOffset.y +=2;
    break;
  }
}