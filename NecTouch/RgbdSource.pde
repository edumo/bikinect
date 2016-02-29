public interface ImageSource {

  void alternativeViewPointDepthToImage();

  void setMirror(Boolean mirrorMode);

  int getWidth();

  int getHeight();

  void update();

  PImage rgbImage();

  int[] depthMap();

  PImage getDepthImage();

  void init(PApplet parent);

  void keyPressed(int key);
  
  void drawGui();
}

public class Kinect2 implements ImageSource {

  int maxD = 4500; // 4.5mx
  int minD = 0;  //  50cm

  KinectPV2 kinect;

  int[] depthZero;

  // BUFFER ARRAY TO CLEAN DE PIXLES
  PImage depthToColorImg;

  PImage depth = null;

  public void init(PApplet parent) {

    depthToColorImg = parent.createImage(512, 424, PImage.RGB);
    depthZero = new int[KinectPV2.WIDTHDepth * KinectPV2.HEIGHTDepth];

    // SET THE ARRAY TO 0s
    for (int i = 0; i < KinectPV2.WIDTHDepth; i++) {
      for (int j = 0; j < KinectPV2.HEIGHTDepth; j++) {
        depthZero[424 * i + j] = 0;
      }
    }

    kinect = new KinectPV2(parent);
    // kinect.enableDepthImg(true);
    kinect.enableInfraredImg(true);
    // kinect.enableColorImg(true);
    // kinect.activateRawDepth(true);
    // kinect.activateRawColor(true);
    kinect.enablePointCloud(true);
    kinect.enableDepthImg(true);
    //context.enablePointCloud(true);
    kinect.init();
  }

  @Override
    public void alternativeViewPointDepthToImage() {
  }

  @Override
    public void setMirror(Boolean mirrorMode) {
    // kinect.
  }

  @Override
    public int getWidth() {
    return KinectPV2.WIDTHDepth;
  }

  @Override
    public int getHeight() {
    return KinectPV2.HEIGHTDepth;
  }

  @Override
    public void update() {
    kinect.setLowThresholdPC(minD);
    kinect.setHighThresholdPC(maxD);
    depth = null;

  }

  @Override
    public PImage rgbImage() {
    return kinect.getInfraredImage();// kinect.getColorImage();
  }

  @Override
    public int[] depthMap() {
    //  if (depth == null) {
    depth = getDepthImage();
    depth.loadPixels();
    //}
    return depth.pixels;
  }

  @Override
    public PImage getDepthImage() {
    // if (depth == null) {
    depth = kinect.getPointCloudDepthImage();
    // }
    return depth;
  }
  
  public void drawGui(){
    
    pushStyle();
   fill(255,255,100);
    text("low / Max threshold & 'u/U' : 'j/J') : "
      + minD + " -> " + maxD, 10, height - 15, 
      450, 20);
    popStyle();
  }

  public void keyPressed(int key) {

    switch(key) {
    case 'j':
      if(minD > 0)
      minD --;
      
      break;  
    case 'J':
     
      minD ++;
      break;  
    case 'u':
      maxD --;
      break;  
    case 'U':
      if(maxD < 4500)
      maxD ++;
      break;
    }
  }
}