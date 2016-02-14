PVector lineIntersection(PVector v1, PVector v2, PVector u1, PVector u2)
{
  return lineIntersection(v1.x, v1.y, v2.x, v2.y, u1.x, u1.y, u2.x, u2.y);
}

PVector lineIntersection(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
{
 
  float bx = x2 - x1;
 
  float by = y2 - y1;
 
  float dx = x4 - x3;
 
  float dy = y4 - y3;
 
 
 
  float b_dot_d_perp = bx*dy - by*dx;
 
 
 
  if(b_dot_d_perp == 0) return null;
 
 
 
  float cx = x3-x1;
 
  float cy = y3-y1;
 
 
 
  float t = (cx*dy - cy*dx) / b_dot_d_perp;
 
 
 
  return new PVector(x1+t*bx, y1+t*by);
 
}


boolean pixelInPoly(PVector[] verts, PVector pos) {
  int i, j;
  boolean c=false;
  int sides = verts.length;
  
  for (i=0,j=sides-1;i<sides;j=i++) {
    if (( ((verts[i].y <= pos.y) && (pos.y < verts[j].y)) || ((verts[j].y <= pos.y) && (pos.y < verts[i].y))) &&
          (pos.x < (verts[j].x - verts[i].x) * (pos.y - verts[i].y) / (verts[j].y - verts[i].y) + verts[i].x)) {
      c = !c;
    }
  }
  return c;
}

PVector pixelIndexToVector(int pixelIndex,int w, int h)
{
  
  return new PVector(pixelIndex%w,floor(pixelIndex/w));
}



float calculatePerspCoord(PVector firstHorizon, PVector secondHorizon, PVector simPoint, PVector firstPoint, PVector secondPoint, PVector upPoint, PVector sideFirstPoint, PVector sideSecondPoint)
{

  PVector bIntersect = lineIntersection(firstHorizon.x, firstHorizon.y, simPoint.x, simPoint.y, firstPoint.x, firstPoint.y, secondPoint.x, secondPoint.y);

  float maxLineDist = PVector.dist(firstPoint, secondPoint);
  if (PVector.dist(bIntersect, firstPoint) > maxLineDist) {
    //todo stop sending here if clip stops
    bIntersect = secondPoint;
  } else if (PVector.dist(bIntersect, secondPoint) > maxLineDist) {
    bIntersect = firstPoint;
  }

  if (upPoint == null) return 0;
  PVector upBIntersect = lineIntersection(bIntersect.x, bIntersect.y, bIntersect.x, bIntersect.y-1, secondHorizon.x, secondHorizon.y, upPoint.x, upPoint.y);
  PVector diagIntersect = lineIntersection(firstPoint.x, firstPoint.y, upPoint.x, upPoint.y, bIntersect.x, bIntersect.y, upBIntersect.x, upBIntersect.y);
  if (diagIntersect == null) return 0;
  PVector targetIntersect = lineIntersection(secondHorizon.x, secondHorizon.y, diagIntersect.x, diagIntersect.y, secondPoint.x, secondPoint.y, upPoint.x, upPoint.y);

  float targetY = PVector.dist(secondPoint, targetIntersect) / PVector.dist(secondPoint, upPoint);

  pushStyle();
  stroke(255, 20, 20, 150);
  fill(255, 20, 20, 150);

  if (showDrawingLines)
  {
    line(firstHorizon.x, firstHorizon.y, bIntersect.x, bIntersect.y);


    //ellipse(upBIntersect.x,upBIntersect.y,5,5);
    line(bIntersect.x, bIntersect.y, diagIntersect.x, diagIntersect.y);

    fill(50, 120, 230, 200);
    stroke(50, 120, 230);
    ellipse(diagIntersect.x, diagIntersect.y, 5, 5);
    line(secondHorizon.x, secondHorizon.y, diagIntersect.x, diagIntersect.y);
  } else
  {
    PVector closerIntersect = lineIntersection(firstHorizon, bIntersect, sideFirstPoint, sideSecondPoint);
   // line(closerIntersect.x, closerIntersect.y, bIntersect.x, bIntersect.y);
  }

  popStyle();

  return targetY;
}


float calculateNonLinearCoord(PVector va, PVector va2, PVector vb, PVector vb2, PVector ve)
{
  float a = (va2.x -va.x)*(vb2.y - vb.y) - (vb2.x - vb.x) * (va2.y - va.y);
  float b = (va2.x - va.x) * (vb.y - ve.y) + (vb2.y - vb.y) * (va.x - ve.x) + (vb2.x - vb.x) * (ve.y - va.y) + (va2.y - va.y) * (ve.x - vb.x);
  float c = (vb.y - ve.y) * (va.x - ve.x) + (vb.x - ve.x) * (ve.y - va.y);

  // ax² + bx + c = 0
  float delta = b*b - 4*a*c;
  float result1 = (-b-sqrt(delta))/(2*a);
  float result2 = (-b+sqrt(delta))/(2*a);

  if (result1 > 0 && result1 < 1)
  {
    return result1;
  } else if (result2 > 0 && result2 < 1)
  {
    return result2;
  }

  return 0;
}


void drawGrid(PVector firstHorizon, PVector secondHorizon, PVector longPoint, PVector midPoint, PVector sideFirstPoint, PVector sideSecondPoint)
{

  pushStyle();

  if (!nonLinearMode)
  {

    stroke(130, 250);
    PVector upPoint = new PVector(midPoint.x, midPoint.y-gridLines);

    for (int i=1; i<gridLines; i++)
    {
      PVector diagIntersect = lineIntersection(longPoint.x, longPoint.y, upPoint.x, upPoint.y, firstHorizon.x, firstHorizon.y, midPoint.x, midPoint.y + (upPoint.y-midPoint.y)*i/gridLines);
      if (diagIntersect == null) break;
      PVector targetEnd = lineIntersection(diagIntersect.x, diagIntersect.y, diagIntersect.x, diagIntersect.y+1, firstHorizon.x, firstHorizon.y, longPoint.x, longPoint.y);
      if (targetEnd == null) break;
      PVector targetBegin = lineIntersection(targetEnd.x, targetEnd.y, secondHorizon.x, secondHorizon.y, sideFirstPoint.x, sideFirstPoint.y, sideSecondPoint.x, sideSecondPoint.y);

      line(targetEnd.x, targetEnd.y, targetBegin.x, targetBegin.y);
    }
  } else
  {
    stroke(200, 100);
    for (int i=1; i<gridLines; i++)
    {
      PVector h1 = new PVector(gbTL.x + i*(gbTR.x-gbTL.x) / gridLines, gbTL.y + i* (gbTR.y-gbTL.y) / gridLines);
      PVector h2 = new PVector(gbBL.x + i*(gbBR.x-gbBL.x) / gridLines, gbBL.y + i*(gbBR.y-gbBL.y) / gridLines);

      PVector v1 = new PVector(gbTL.x + i* (gbBL.x-gbTL.x) / gridLines, gbTL.y + i*(gbBL.y-gbTL.y) / gridLines);
      PVector v2 = new PVector(gbTR.x + i* (gbBR.x-gbTR.x) / gridLines, gbTR.y + i*(gbBR.y-gbTR.y) / gridLines);

      /*if(v1.x > v2.x || h1.y > h2.y)
       {
       stroke(200,50,50,100);
       }else{
       stroke(200,100);
       }*/

      line(h1.x, h1.y, h2.x, h2.y);
      line(v1.x, v1.y, v2.x, v2.y);
    }
  }


  popStyle();
}

void drawRepere(PVector horizon, PVector longPoint, PVector midPoint, PVector upPoint, PVector upMPoint)
{
  if (upMPoint == null) return;

  ellipse(upPoint.x, upPoint.y, 5, 5);
  pushStyle(); 
  stroke(200, 100);
  line(longPoint.x, longPoint.y, upPoint.x, upPoint.y);
  line(horizon.x, horizon.y, upPoint.x, upPoint.y);

  ellipse(upMPoint.x, upMPoint.y, 5, 5);
  line(midPoint.x, midPoint.y, upMPoint.x, upMPoint.y);
  stroke(250, 250, 20, 100);
  line(longPoint.x, longPoint.y, upMPoint.x, upMPoint.y);
  popStyle();
}