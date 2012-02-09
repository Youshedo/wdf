import processing.core.*; 
import processing.xml.*; 

import java.applet.*; 
import java.awt.Dimension; 
import java.awt.Frame; 
import java.awt.event.MouseEvent; 
import java.awt.event.KeyEvent; 
import java.awt.event.FocusEvent; 
import java.awt.Image; 
import java.io.*; 
import java.net.*; 
import java.text.*; 
import java.util.*; 
import java.util.zip.*; 
import java.util.regex.*; 

public class test extends PApplet {

int r;
int g;
int b;
int c;
 
int numOfArcs = 20;
int strokeWidth = 5;
int rDist = 10;
LightArc lArcsMobile[] = new LightArc[numOfArcs];
 
public void setup(){
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
 
public void draw(){
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
  int arcColour;
  int direction;
   
  LightArc(int tempDia, int c){
    arcStartDeg = random(0,360);
    arcLength = random(0,360);
    arcStopDeg = arcStartDeg + arcLength;
    dia = tempDia;
    arcColour = c;
    direction = round(random(0,1));
    speed = random(0.1f,1);
  }
   
  public void update(){
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

  static public void main(String args[]) {
    PApplet.main(new String[] { "--bgcolor=#FFFFFF", "test" });
  }
}
