public interface RgbdSource {

  void alternativeViewPointDepthToImage();

  void setMirror(Boolean mirrorMode);

  int depthWidth();

  int depthHeight();

  void update();

  PImage rgbImage();

  int[] depthMap();

  PImage depthImage();

  void init(PApplet parent);

}


public class Kinect2 implements RgbdSource {

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
  public int depthWidth() {
    return KinectPV2.WIDTHDepth;
  }

  @Override
  public int depthHeight() {
    return KinectPV2.HEIGHTDepth;
  }

  @Override
  public void update() {

    depth = null;
    // float[] mapDCT = kinect.getMapDepthToColor();

    // get the raw data from depth and color
    // int[] colorRaw = kinect.getRawColor();
    // int[] depthRaw = kinect.getRawDepth();

    // clean de pixels
    // PApplet.arrayCopy(depthZero, depthToColorImg.pixels);
    //
    // int count = 0;
    // depthToColorImg.loadPixels();
    // for (int i = 0; i < KinectPV2.WIDTHDepth; i++) {
    // for (int j = 0; j < KinectPV2.HEIGHTDepth; j++) {
    //
    // // incoming pixels 512 x 424 with position in 1920 x 1080
    // float valX = mapDCT[count * 2 + 0];
    // float valY = mapDCT[count * 2 + 1];
    //
    // // maps the pixels to 512 x 424, not necessary but looks better
    // int valXDepth = (int) ((valX / 1920.0f) * 512.0f);
    // int valYDepth = (int) ((valY / 1080.0f) * 424.0f);
    //
    // int valXColor = (int) (valX);
    // int valYColor = (int) (valY);
    //
    // if (valXDepth >= 0 && valXDepth < 512 && valYDepth >= 0
    // && valYDepth < 424 && valXColor >= 0
    // && valXColor < 1920 && valYColor >= 0
    // && valYColor < 1080) {
    // int colorPixel = colorRaw[valYColor * 1920 + valXColor];
    // // color colorPixel = depthRaw[valYDepth*512 + valXDepth];
    // depthToColorImg.pixels[valYDepth * 512 + valXDepth] = colorPixel;
    // }
    // count++;
    // }
    // }
    // depthToColorImg.updatePixels();
  }

  @Override
  public PImage rgbImage() {
    return kinect.getInfraredImage();// kinect.getColorImage();
  }

  @Override
  public int[] depthMap() {
    if (depth == null) {
      depth = kinect.getPointCloudDepthImage();
      depth.loadPixels();
    }
    return depth.pixels;
  }

  @Override
  public PImage depthImage() {
    if (depth != null) {
      depth = kinect.getPointCloudDepthImage();
    }
    return depth;
  }

}