public class CVDiewald implements ComputerVision {

  PImage sample_img;

  int size_x, size_y;
  BlobDetector blob_detector;
  BoundingBox detection_area;
  int detection_resolution = 1;
  boolean draw_blobs_boundingsbox = true;
  boolean draw_filled_blobs = true;
  BLOBable_GRADIENT bable_GRADIENT;

  public void init(PApplet parent, int w, int h) {

    size_x = w;
    size_y = h;

    blob_detector = new BlobDetector(size_x, size_y);
    blob_detector.setResolution(detection_resolution);
    blob_detector.computeContours(true);
    blob_detector.computeBlobPixels(!true);
    blob_detector.setMinMaxPixels(10 * 10, size_x * size_y);

    bable_GRADIENT = new BLOBable_GRADIENT(parent, sample_img);
    blob_detector.setBLOBable(bable_GRADIENT);

    detection_area = new BoundingBox(0, 0, size_x, size_y);
    blob_detector.setDetectingArea(detection_area);

  }

  
  public BiBlob[] findBlobs(PImage blobsImage) {

    blob_detector.setResolution(detection_resolution);
    blob_detector.setMinMaxPixels(50, size_x * size_y);
    // update the blob-detector with the new pixelvalues
    bable_GRADIENT.setImg_(blobsImage);
    blob_detector.update();

    // get a list of all the blobs
    ArrayList<Blob> blob_list = blob_detector.getBlobs();

    BiBlob[] blobs = new BiBlob[blob_list.size()];

    // iterate through the blob_list
    for (int blob_idx = 0; blob_idx < blob_list.size(); blob_idx++) {

      // get the current blob from the blob-list
      Blob blob = blob_list.get(blob_idx);
      BiBlob blobDiewald = new BlobDiewald(blob);
      blobs[blob_idx] = blobDiewald;
    }

    return blobs;
  }
}

public final class BLOBable_GRADIENT implements BLOBable {
    int width_, height_;
    private float hsb_[] = new float[3];
    private float mousex_val_, mousey_val_;
    private String name_;
    private PApplet papplet_;
    private PImage img_;

    public void setImg_(PImage img_) {
      this.img_ = img_;
    }

    public BLOBable_GRADIENT(PApplet papplet, PImage img) {
      papplet_ = papplet;
      img_ = img;
    }

    // @Override
    public final void init() {
      name_ = this.getClass().getSimpleName();
    }

    // @Override
    public final void updateOnFrame(int width, int height) {
      width_ = width;
      height_ = height;
//      mousex_val_ = PApplet
//          .map(papplet_.mouseX, 0, papplet_.width, 0, 99);
//      mousey_val_ = PApplet.map(papplet_.mouseY, 0, papplet_.height, 0,
//          360);
//      if (mousex_val_ > 98)
//        mousex_val_ = 98;
      // System.out.println("MY NAME IS: "
      // +this.getClass().getSimpleName());
    }

    // @Override
    public final boolean isBLOBable(int pixel_index, int x, int y) {
      if (PixelColor.red(img_.pixels[pixel_index]) > 0) {
        return true;
      } else {
        return false;
      }
    }
  }

public class BlobDiewald implements BiBlob {

  Blob blob;

  PVector centroid = null;

  PVector[] points = null;

  BoundingBox bb;

  public BlobDiewald(Blob blob) {
    this.blob = blob;
    // get the list of all the contours from the current blob
    ArrayList<Contour> contour_list = blob.getContours();

//    if (contour_list.size() > 1)
//      System.out.println("VIENE MAS DE UN CONTORNO CONTROLAR");
    // iterate through the contour_list
    for (int contour_idx = 0; contour_idx < 1; contour_idx++) {

      // get the current contour from the contour-list
      Contour contour = contour_list.get(contour_idx);

      // get the current boundingbox from the current contour
      bb = contour.getBoundingBox();

      centroid = new PVector(bb.xCenter(), bb.yCenter());

      List<Pixel> pixels = contour.getPixels();

      points = new PVector[pixels.size()];

      for (int i = 0; i < pixels.size(); i++) {
        Pixel p = pixels.get(i);
        points[i] = new PVector(p.x_, p.y_);
      }

      // handle the first contour (outer contour = contour_idx == 0)
      // different to the inner contours
      if (contour_idx == 0) {

        // draw the boundingbox and blob-id as text
        // if (draw_blobs_boundingsbox) {
        // // drawBoundingBox(bb, color(0, 200, 200), 1);
        // // fill(0, 200, 200);
        // // text("blob[" + blob_idx + "]", bb.xMin(), bb.yMin()
        // // - textDescent() * 2);
        // }

        // draw the contour
        // drawContour(contour.getPixels(), color(255, 0, 0),
        // color(255, 0, 255, 100), draw_filled_blobs, 1);
      } else {
        // draw the inner contours, if they have more than 20
        // vertices
        if (contour.getPixels().size() > 20) {
          // drawContour(contour.getPixels(), color(255, 150, 0),
          // color(0, 100), false, 1);
        }
      }
    }
  }

  @Override
  public PVector getCentroid() {

    return centroid;
  }

  @Override
  public PVector[] getPoints() {
    // TODO Auto-generated method stub
    return points;
  }

  @Override
  public float getArea() {
    return bb.xMax() * bb.yMax();
  }

}