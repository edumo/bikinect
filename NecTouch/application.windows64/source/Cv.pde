
public interface ComputerVision {

  void init(PApplet parent, int w, int h);

  BiBlob[] findBlobs(PImage blobsImage);

}

public interface BiBlob {

  PVector getCentroid();

  PVector[] getPoints();

  float getArea();
  
  

}