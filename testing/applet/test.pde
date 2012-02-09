int r;
int g;
int b;
color c;
 
int numOfArcs = 20;
int strokeWidth = 5;
int rDist = 10;
LightArc lArcsMobile[] = new LightArc[numOfArcs];
 
void setup(){
  size(800,450);
  noFill();
  smooth();
  strokeWeight(strokeWidth);
  strokeCap(SQUARE);
  for(int i=0;i<numOfArcs;i++){
    r = 0;
    g = 255;
    b = 0;
    c = color(r,g,b);
    rDist = i * 20;
    lArcsMobile[i] = new LightArc(rDist,c);
  }
}
 
void draw(){
  background(0);
  for(int j = 0; j < numOfArcs; j++){
    lArcsMobile[j].update();
  }
}
 
class LightArc {
  int randAlph;
  int dia;
  float arcStartDeg;
  float arcStopDeg;
  float arcStartRad;
  float arcStopRad;
  float arcLength;
  int xPos;
  int yPos;
  float speed;
  color arcColour;
  int direction;
   
  LightArc(int tempDia, color c){
    arcStartDeg = random(0,360);
    arcLength = random(0,360);
    arcStopDeg = arcStartDeg + arcLength;
    dia = tempDia;
    arcColour = c;
    direction = round(random(0,1));
    speed = random(0.1,1);
  }
   
  void update(){
    arcStartRad = radians(arcStartDeg);
    arcStopRad = radians(arcStopDeg);
    stroke(arcColour);
    arc(width/2,height/2,dia,dia,arcStartRad,arcStopRad);
    if(direction > 0){
      arcStartDeg+= speed;
      arcStopDeg+= speed;
    } else {
      arcStartDeg-= speed;
      arcStopDeg-= speed;
    }   
  }
}

