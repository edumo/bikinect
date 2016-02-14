class TouchPoint extends PVector {

    public int id;
    TouchPoint lastPoint;
    float speedX, speedY;
    float speed;
    float acc;
    Boolean linked;
    float weight;

    String currentState;

    PVector reelCoord;

    // A�ado el array de puntos
    PVector[] vertex = null;

    boolean object = false;
    boolean touch = false;

    Float angle = null;

    KalmanFacade kalmanFacade;

    public TouchPoint(float tx, float ty, float tweight,
        PVector reelCoordVec, boolean isLast, PVector[] vertex) {

      x = tx;
      y = ty;
      weight = tweight;
      linked = false;
      reelCoord = reelCoordVec;
      this.vertex = vertex;

      if (!isLast) {
        setLastPoint(new TouchPoint(tx, ty, 10, null, true, vertex));
      }
    }

    public void buildKalman() {
      kalmanFacade = new KalmanFacade(x, y);
    }

    public void updateKalman(KalmanFacade kalmanFacade) {
      this.kalmanFacade = kalmanFacade;
      kalmanFacade.update(x, y);
    }

    public void setLastPoint(TouchPoint lp) {
      lastPoint = lp;
      speedX = x - lastPoint.x;
      speedY = y - lastPoint.y;

      float lastSpeed = lastPoint.speed;
      speed = PVector.dist(this, lastPoint);
      acc = speed - lastSpeed;
    }

    public void setState(String state) {
      currentState = state;

      if (state.equals("new")) {
        globalTouchPointIndex++;
        this.id = globalTouchPointIndex;
      }
    }

    public void drawPointReel(int c) {
      pushStyle();
      noFill();
      noStroke();
      stroke(c);
      ellipse(reelCoord.x, reelCoord.y, weight / 5, weight / 5);
      popStyle();
    }
  }