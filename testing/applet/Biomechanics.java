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

public class Biomechanics extends PApplet {

float depth = 256;
float cubeSize = 2;
float density = 0.01f/pow(cubeSize,3);
int pointSampling = round(cubeSize);
float spring = 0.01f;// cube surface
float spring2 = 0.3f;// joint translation
float spring3 = 50;// joint rotation
float damping = 0.005f;// cube surface
float damping2 = 0.1f;// joint translation
float damping3 = 1;// joint rotation
float damping4 = 8;// global rotation
float grav = 0.01f;
int iterationsPerFrame = 3;
float randVel = PI/32;
cube[] cubes;
joint[] joints;
hemisphere[] hemispheres;
public void setup(){
  size(400,300,P3D);
  ortho(-width/2, width/2, -height/2, height/2, -10, 10);
  cubes = new cube[10];
  joints = new joint[9];
  hemispheres = new hemisphere[66];
  reset();
  noStroke();
  fill(0xffFF8800);
  mouseGripper = new mouseJoint();
}
public void reset(){
  cube head = new cube(5.5f,8,6,11,6,0);
  cube torso = new cube(10,18,6,11,20,0);
  cube Rarm = new cube(4,10,4,3,17,0);
  cube Larm = new cube(4,10,4,19,17,0);
  cube Rforearm = new cube(4,12,4,3,29,0);
  cube Lforearm = new cube(4,12,4,19,29,0);
  cube Rthigh = new cube(4.5f,12,4.5f,8,36,0);
  cube Lthigh = new cube(4.5f,12,4.5f,14,36,0);
  cube Rcalf = new cube(4,16,4,8,51,0);
  cube Lcalf = new cube(4,16,4,14,51,0);
  cubes[0] = head;
  cubes[1] = torso;
  cubes[2] = Rarm;
  cubes[3] = Larm;
  cubes[4] = Rforearm;
  cubes[5] = Lforearm;
  cubes[6] = Rthigh;
  cubes[7] = Lthigh;
  cubes[8] = Rcalf;
  cubes[9] = Lcalf;
  joint neck = new joint(torso,head,0,-10,1,0,4,1);
  joint Rshoulder = new joint(torso,Rarm,-8,-6,0,0,-3,0);
  joint Lshoulder = new joint(torso,Larm,8,-6,0,0,-3,0);
  joint Relbow = new joint(Rarm,Rforearm,0,8,0,0,-4,0);
  joint Lelbow = new joint(Larm,Lforearm,0,8,0,0,-4,0);
  joint Rhip = new joint(torso,Rthigh,-3,12,0,0,-4,0);
  joint Lhip = new joint(torso,Lthigh,3,12,0,0,-4,0);
  joint Rknee = new joint(Rthigh,Rcalf,0,9,0,0,-6,0);
  joint Lknee = new joint(Lthigh,Lcalf,0,9,0,0,-6,0);
  joints[0] = neck;
  joints[1] = Rshoulder;
  joints[2] = Lshoulder;
  joints[3] = Relbow;
  joints[4] = Lelbow;
  joints[5] = Rhip;
  joints[6] = Lhip;
  joints[7] = Rknee;
  joints[8] = Lknee;
  hemispheres[0] = new hemisphere(torso,head,2,-1,0,0,-1,0);
  hemispheres[1] = new hemisphere(torso,head,-2,-1,0,0,-1,0);
  hemispheres[2] = new hemisphere(torso,head,0,-1,1,0,-1,0);
  hemispheres[3] = new hemisphere(torso,head,0,-1,-1,0,-1,0);
  hemispheres[4] = new hemisphere(torso,head,1,0,-1,0,0,-1);
  hemispheres[5] = new hemisphere(torso,head,-1,0,-1,0,0,-1);
  hemispheres[6] = new hemisphere(Rarm,torso,0,0,1,-1,0,0);
  hemispheres[7] = new hemisphere(Rarm,torso,0,0,-1,-1,0,0);
  hemispheres[8] = new hemisphere(Rarm,torso,-1,0,0,-1,0,0);
  hemispheres[9] = new hemisphere(Rarm,torso,0,1,0,-1,0,0);
  hemispheres[10] = new hemisphere(torso,Rarm,0,1,-1,0,1,0);
  hemispheres[11] = new hemisphere(torso,Rarm,0,1,-1,1,0,0);
  hemispheres[12] = new hemisphere(Larm,torso,0,0,1,1,0,0);
  hemispheres[13] = new hemisphere(Larm,torso,0,0,-1,1,0,0);
  hemispheres[14] = new hemisphere(Larm,torso,1,0,0,1,0,0);
  hemispheres[15] = new hemisphere(Larm,torso,0,1,0,1,0,0);
  hemispheres[16] = new hemisphere(torso,Larm,0,1,-1,0,1,0);
  hemispheres[17] = new hemisphere(torso,Larm,0,1,-1,-1,0,0);
  hemispheres[18] = new hemisphere(Rarm,Rforearm,0,1,0,0,1,0);
  hemispheres[19] = new hemisphere(Rarm,Rforearm,0,0,-1,0,1,0);
  hemispheres[20] = new hemisphere(Rarm,Rforearm,1,0,-1,0,1,0);
  hemispheres[21] = new hemisphere(Rarm,Rforearm,-1,0,-1,0,1,0);
  hemispheres[22] = new hemisphere(Rarm,Rforearm,1,0,0,0,0,-1);
  hemispheres[23] = new hemisphere(Rarm,Rforearm,-1,0,0,0,0,-1);
  hemispheres[24] = new hemisphere(Rarm,Rforearm,0,0,-1,0,0,-1);
  hemispheres[25] = new hemisphere(Rarm,Rforearm,-1,0,0,-1,0,0);
  hemispheres[26] = new hemisphere(Larm,Lforearm,0,1,0,0,1,0);
  hemispheres[27] = new hemisphere(Larm,Lforearm,0,0,-1,0,1,0);
  hemispheres[28] = new hemisphere(Larm,Lforearm,-1,0,-1,0,1,0);
  hemispheres[29] = new hemisphere(Larm,Lforearm,1,0,-1,0,1,0);
  hemispheres[30] = new hemisphere(Larm,Lforearm,1,0,0,0,0,-1);
  hemispheres[31] = new hemisphere(Larm,Lforearm,-1,0,0,0,0,-1);
  hemispheres[32] = new hemisphere(Larm,Lforearm,0,0,-1,0,0,-1);
  hemispheres[33] = new hemisphere(Rarm,Rforearm,1,0,0,1,0,0);
  hemispheres[34] = new hemisphere(torso,Rthigh,-1,0,0,0,1,0);
  hemispheres[35] = new hemisphere(torso,Rthigh,0,1,0,0,1,0);
  hemispheres[36] = new hemisphere(torso,Rthigh,0,0,-1,0,1,0);
  hemispheres[37] = new hemisphere(torso,Rthigh,1,1,-1,0,1,0);
  hemispheres[38] = new hemisphere(torso,Rthigh,-1,0,0,0,0,1);
  hemispheres[39] = new hemisphere(torso,Rthigh,1,0,0,0,0,1);
  hemispheres[40] = new hemisphere(torso,Rthigh,0,0,1,0,0,1);
  hemispheres[41] = new hemisphere(torso,Rthigh,-1,0,0,-1,0,0);
  hemispheres[42] = new hemisphere(torso,Lthigh,1,0,0,0,1,0);
  hemispheres[43] = new hemisphere(torso,Lthigh,0,1,0,0,1,0);
  hemispheres[44] = new hemisphere(torso,Lthigh,0,0,-1,0,1,0);
  hemispheres[45] = new hemisphere(torso,Lthigh,-1,1,-1,0,1,0);
  hemispheres[46] = new hemisphere(torso,Lthigh,1,0,0,0,0,1);
  hemispheres[47] = new hemisphere(torso,Lthigh,-1,0,0,0,0,1);
  hemispheres[48] = new hemisphere(torso,Lthigh,0,0,1,0,0,1);
  hemispheres[49] = new hemisphere(torso,Lthigh,1,0,0,1,0,0);
  hemispheres[50] = new hemisphere(Rthigh,Rcalf,0,1,0,0,1,0);
  hemispheres[51] = new hemisphere(Rthigh,Rcalf,0,0,1,0,1,0);
  hemispheres[52] = new hemisphere(Rthigh,Rcalf,1,0,1,0,1,0);
  hemispheres[53] = new hemisphere(Rthigh,Rcalf,-1,0,1,0,1,0);
  hemispheres[54] = new hemisphere(Rthigh,Rcalf,1,0,0,0,0,1);
  hemispheres[55] = new hemisphere(Rthigh,Rcalf,-1,0,0,0,0,1);
  hemispheres[56] = new hemisphere(Rthigh,Rcalf,0,0,1,0,0,1);
  hemispheres[57] = new hemisphere(Rthigh,Rcalf,-1,0,0,-1,0,0);
  hemispheres[58] = new hemisphere(Lthigh,Lcalf,0,1,0,0,1,0);
  hemispheres[59] = new hemisphere(Lthigh,Lcalf,0,0,1,0,1,0);
  hemispheres[60] = new hemisphere(Lthigh,Lcalf,1,0,1,0,1,0);
  hemispheres[61] = new hemisphere(Lthigh,Lcalf,-1,0,1,0,1,0);
  hemispheres[62] = new hemisphere(Lthigh,Lcalf,1,0,0,0,0,1);
  hemispheres[63] = new hemisphere(Lthigh,Lcalf,-1,0,0,0,0,1);
  hemispheres[64] = new hemisphere(Lthigh,Lcalf,0,0,1,0,0,1);
  hemispheres[65] = new hemisphere(Lthigh,Lcalf,1,0,0,1,0,0);
  for(int i=0;i<cubes.length;i++){
    cubes[i].loc.add(width/2-11*cubeSize,
      height/2-24*cubeSize,depth/2);
  }
}
public void draw(){
  if(frameCount%60==0){println(frameRate);}
  background(0xff8800FF);
  directionalLight(255, 255, 255, -1, 1, -1);
  directionalLight(128, 128, 128, 1, -1, 1);
  for(int m=0;m<iterationsPerFrame;m++){
    for(int i=0;i<cubes.length;i++){
      for(int j=0;j<cubes.length;j++){
        cubes[i].collide(cubes[j]);
      }
    }
    float Esum = 0;
    for(int i=0;i<joints.length;i++){
      joints[i].update();
    }
    for(int i=0;i<hemispheres.length;i++){
      hemispheres[i].update();
    }
    if(mousePressed){
      mouseGripper.update();
    }
    for(int i=0;i<cubes.length;i++){
      cubes[i].walls();
      cubes[i].update();
    }
  }
  for(int i=0;i<cubes.length;i++){
    PVector[][][] verts = new PVector[2][2][2];
    PVector[] verts2 = new PVector[8];
    for(int a=0;a<=1;a++){
      for(int b=0;b<=1;b++){
        for(int c=0;c<=1;c++){
          verts[a][b][c] = new PVector(a-0.5f,b-0.5f,c-0.5f);
        }
      }
    }
    verts2[0] = verts[0][0][0];
    verts2[1] = verts[1][0][0];
    verts2[2] = verts[0][1][0];
    verts2[3] = verts[0][0][1];
    verts2[4] = verts[1][1][0];
    verts2[5] = verts[1][0][1];
    verts2[6] = verts[0][1][1];
    verts2[7] = verts[1][1][1];
    for(int j=0;j<8;j++){
      verts2[j].x *= cubes[i].size.x;
      verts2[j].y *= cubes[i].size.y;
      verts2[j].z *= cubes[i].size.z;
      verts2[j] = cubes[i].ori.toWorld(verts2[j]);
      verts2[j].add(cubes[i].loc);
    }
    verts[0][0][0] = verts2[0];
    verts[1][0][0] = verts2[1];
    verts[0][1][0] = verts2[2];
    verts[0][0][1] = verts2[3];
    verts[1][1][0] = verts2[4];
    verts[1][0][1] = verts2[5];
    verts[0][1][1] = verts2[6];
    verts[1][1][1] = verts2[7];
    int[] vertA = new int[6*2*3];
    int[] vertB = new int[6*2*3];
    int[] vertC = new int[6*2*3];
    
    vertA[0 ] =  1; vertB[0 ] =  1; vertC[0 ] =  1;
    vertA[1 ] =  1; vertB[1 ] = -1; vertC[1 ] =  1;
    vertA[2 ] =  1; vertB[2 ] =  1; vertC[2 ] = -1;
    
    vertA[3 ] =  1; vertB[3 ] = -1; vertC[3 ] = -1;
    vertA[4 ] =  1; vertB[4 ] = -1; vertC[4 ] =  1;
    vertA[5 ] =  1; vertB[5 ] =  1; vertC[5 ] = -1;
    
    vertA[6 ] = -1; vertB[6 ] =  1; vertC[6 ] =  1;
    vertA[7 ] = -1; vertB[7 ] = -1; vertC[7 ] =  1;
    vertA[8 ] = -1; vertB[8 ] =  1; vertC[8 ] = -1;
    
    vertA[9 ] = -1; vertB[9 ] = -1; vertC[9 ] = -1;
    vertA[10] = -1; vertB[10] = -1; vertC[10] =  1;
    vertA[11] = -1; vertB[11] =  1; vertC[11] = -1;
    
    vertA[12] =  1; vertB[12] =  1; vertC[12] =  1;
    vertA[13] =  1; vertB[13] =  1; vertC[13] = -1;
    vertA[14] = -1; vertB[14] =  1; vertC[14] =  1;
    
    vertA[15] = -1; vertB[15] =  1; vertC[15] = -1;
    vertA[16] =  1; vertB[16] =  1; vertC[16] = -1;
    vertA[17] = -1; vertB[17] =  1; vertC[17] =  1;
    
    vertA[18] =  1; vertB[18] = -1; vertC[18] =  1;
    vertA[19] =  1; vertB[19] = -1; vertC[19] = -1;
    vertA[20] = -1; vertB[20] = -1; vertC[20] =  1;
    
    vertA[21] = -1; vertB[21] = -1; vertC[21] = -1;
    vertA[22] =  1; vertB[22] = -1; vertC[22] = -1;
    vertA[23] = -1; vertB[23] = -1; vertC[23] =  1;
    
    vertA[24] =  1; vertB[24] =  1; vertC[24] =  1;
    vertA[25] = -1; vertB[25] =  1; vertC[25] =  1;
    vertA[26] =  1; vertB[26] = -1; vertC[26] =  1;
    
    vertA[27] = -1; vertB[27] = -1; vertC[27] =  1;
    vertA[28] = -1; vertB[28] =  1; vertC[28] =  1;
    vertA[29] =  1; vertB[29] = -1; vertC[29] =  1;
    
    vertA[30] =  1; vertB[30] =  1; vertC[30] = -1;
    vertA[31] = -1; vertB[31] =  1; vertC[31] = -1;
    vertA[32] =  1; vertB[32] = -1; vertC[32] = -1;
    
    vertA[33] = -1; vertB[33] = -1; vertC[33] = -1;
    vertA[34] = -1; vertB[34] =  1; vertC[34] = -1;
    vertA[35] =  1; vertB[35] = -1; vertC[35] = -1;
    
    for(int j=0;j<6*2*3;j++){
      if(vertA[j]==-1){vertA[j]=0;}
      if(vertB[j]==-1){vertB[j]=0;}
      if(vertC[j]==-1){vertC[j]=0;}
    }
    beginShape(TRIANGLES);
    for(int j=0;j<6*2*3;j++){
      vertex(verts[vertA[j]][vertB[j]][vertC[j]].x,
        verts[vertA[j]][vertB[j]][vertC[j]].y,
        -verts[vertA[j]][vertB[j]][vertC[j]].z);
    }
    endShape();
  }
}
mouseJoint mouseGripper;
public void mousePressed(){
  PVector mouse = new PVector(mouseX,mouseY);
  boolean done = false;
  mouse.z = depth;
  while(mouse.z>0&&done==false){
    for(int i=0;i<cubes.length;i++){
      if(cubes[i].isInCube(mouse)){
        mouseGripper = new mouseJoint(cubes[i],
          cubes[i].ori.toOri(PVector.sub(mouse,cubes[i].loc)));
      }
    }
    mouse.z -= cubeSize;
  }
}
public void mouseReleased(){
  mouseGripper.nullify();
}
public void keyPressed(){
  if(key==' '){reset();}
}
class cube{
  PVector loc;
  PVector vel;
  PVector force;
  orient ori;
  PVector angVel;
  PVector torque;
  PVector size;
  float mass;
  PVector momentOfIniertia;
  cube(float w,float h,float d,
       float x,float y,float z){
    loc = new PVector(x*cubeSize,y*cubeSize,z*cubeSize);
    vel = new PVector();
    force = new PVector();
    ori = new orient();
    angVel = new PVector(
      random(-randVel,randVel),
      random(-randVel,randVel),
      random(-randVel,randVel));
    torque = new PVector();
    size = new PVector(w*cubeSize,h*cubeSize,d*cubeSize);
    mass = size.x*size.y*size.z*density;
    momentOfIniertia = new PVector(
      mass*(sq(size.y)+sq(size.z))/12f,
      mass*(sq(size.x)+sq(size.z))/12f,
      mass*(sq(size.x)+sq(size.y))/12f);
  }
  public void applyForce(PVector F,PVector R){
    force.add(F);
    torque.add(PVector.sub(R,loc).cross(F));
  }
  public boolean isInCube(PVector r){
    PVector R = ori.toOri(PVector.sub(r,loc));
    boolean val = false;
    if(abs(R.x)<size.x/2
      &&abs(R.y)<size.y/2
      &&abs(R.z)<size.z/2){val = true;}
    return val;
  }
  public PVector forceField(PVector r){
    PVector R = ori.toOri(PVector.sub(r,loc));
    float fpx = -spring*(R.x-size.x/2);
    PVector F = new PVector(fpx,0,0);
    float fnx = -spring*(R.x+size.x/2);
    if(abs(fnx)<F.mag()){F=new PVector(fnx,0,0);}
    float fpy = -spring*(R.y-size.y/2);
    if(abs(fpy)<F.mag()){F=new PVector(0,fpy,0);}
    float fny = -spring*(R.y+size.y/2);
    if(abs(fny)<F.mag()){F=new PVector(0,fny,0);}
    float fpz = -spring*(R.z-size.z/2);
    if(abs(fpz)<F.mag()){F=new PVector(0,0,fpz);}
    float fnz = -spring*(R.z+size.z/2);
    if(abs(fnz)<F.mag()){F=new PVector(0,0,fnz);}
    return ori.toWorld(F);
  }
  public PVector[] testPoints(){
    int a = ceil(size.x/pointSampling);
    int b = ceil(size.y/pointSampling);
    int c = ceil(size.z/pointSampling);
    PVector[] tp = new PVector[
//      (a+1)*(b+1)*(c+1)-(a-1)*(b-1)*(c-1)];
      (a+1)*(b+1)*(c+1)];
    int index = 0;
    for(int i=0;i<=a;i++){
      for(int j=0;j<=b;j++){
        for(int k=0;k<=c;k++){
//          if(i==0||i==a||j==0||j==b||k==0||k==c){
            tp[index] = new PVector(
              size.x*(i*1f/a-0.5f),
              size.y*(j*1f/b-0.5f),
              size.z*(k*1f/c-0.5f));
            tp[index] = PVector.add(ori.toWorld(tp[index]),loc);
            index++;
//          }
        }
      }
    }
    return tp;
  }
  public PVector totalVelAt(PVector R){
    return PVector.add(vel,angVel.cross(PVector.sub(R,loc)));
  }
  public void collide(cube C){
    PVector dx = PVector.sub(C.loc,loc);
    if(dx.mag()<C.size.mag()/2+size.mag()/2){
      PVector Fsum = new PVector();
      int count = 0;
      PVector[] tp = testPoints();
      for(int i=0;i<tp.length;i++){
        PVector R = tp[i];
        if(C.isInCube(R)){
          PVector F = PVector.add(
            C.forceField(R),PVector.mult(PVector.sub(
            C.totalVelAt(R),totalVelAt(R)),damping));
          applyForce(F,R);
          F.mult(-1);
          C.applyForce(F,R);
        }
      }
    }
  }
  public void update(){
//    force.limit(8);
    vel.add(PVector.mult(force,1f/mass));
    force = new PVector(0,grav*mass,0);
    loc.add(vel);
//    torque.limit(PI/6);
    torque = ori.toOri(torque);
    torque.x /= momentOfIniertia.x;
    torque.y /= momentOfIniertia.y;
    torque.z /= momentOfIniertia.z;
    torque = ori.toWorld(torque);
    angVel.add(torque);
    angVel.limit(PI/16);
    torque = new PVector();
    ori.spin(angVel);
  }
  public boolean wallHit(){
    boolean val = false;
    float testL = size.mag()/2;
    if(loc.x<testL){val=true;}
    if(loc.x>width-testL){val=true;}
    if(loc.y<testL){val=true;}
    if(loc.y>height-testL){val=true;}
    if(loc.z<testL){val=true;}
    if(loc.z>depth-testL){val=true;}
    return val;
  }
  public PVector wallFeild(PVector R){
    PVector F = new PVector();
    boolean stick = false;
    if(R.x<0){F.x=-R.x*spring;stick=true;}
    if(R.x>width){F.x=-(R.x-width)*spring;stick=true;}
    if(R.y<0){F.y=-R.y*spring;stick=true;}
    if(R.y>height){F.y=-(R.y-height)*spring;stick=true;}
    if(R.z<0){F.z=-R.z*spring;}
    if(R.z>depth){F.z=-(R.z-depth)*spring;}
    if(stick){F.sub(PVector.mult(totalVelAt(R),damping));}
    return F;
  }
  public void walls(){
    if(wallHit()){
      PVector[] tp = testPoints();
      for(int i=0;i<tp.length;i++){
        PVector R = tp[i];
        applyForce(wallFeild(R),R);
      }
    }
  }
}
class joint{
  cube A;
  cube B;
  PVector r1;
  PVector r2;
  joint(cube Ao,cube Bo,
    float x1,float y1,float z1,
    float x2,float y2,float z2){
    A = Ao;
    B = Bo;
    r1 = new PVector(x1,y1,z1);
    r1.mult(cubeSize);
    r2 = new PVector(x2,y2,z2);
    r2.mult(cubeSize);
  }
  public void update(){
    PVector R1 = PVector.add(A.loc,A.ori.toWorld(r1));
    PVector R2 = PVector.add(B.loc,B.ori.toWorld(r2));
    PVector dx = PVector.sub(R2,R1);
    float F = dx.mag()*spring2;
    dx.normalize();
    float D = dx.dot(PVector.sub(
      B.totalVelAt(R2),A.totalVelAt(R1)))*damping2;
    dx.mult(F+D);
    A.applyForce(dx,R1);
    dx.mult(-1);
    B.applyForce(dx,R2);
    PVector dAngVel = PVector.sub(B.angVel,A.angVel);
    dAngVel.mult(damping4);
    A.torque.add(dAngVel);
    B.torque.sub(dAngVel);
  }
}
class mouseJoint{
  boolean active;
  cube held;
  PVector loc;
  mouseJoint(cube h,PVector L){
    held = h;
    loc = L;
    active = true;
  }
  mouseJoint(){
    active = false;
  }
  public void nullify(){
    active = false;
  }
  public void update(){
    if(active){
      PVector mouse = new PVector(mouseX,mouseY);
      PVector pmouse = new PVector(pmouseX,pmouseY);
      PVector vmouse = PVector.sub(mouse,pmouse);
      vmouse.mult(1f/iterationsPerFrame);
      PVector R = PVector.add(held.loc,held.ori.toWorld(loc));
      mouse.z = R.z;
      PVector dx = PVector.sub(mouse,R);
      float F = dx.mag()*0.05f;
      dx.normalize();
      float D = dx.dot(PVector.sub(
        vmouse,held.totalVelAt(R)))*0.5f;
      dx.mult(F+D);
      held.applyForce(dx,R);
    }
  }
}
class hemisphere{
  cube A;
  cube B;
  PVector r1;
  PVector r2;
  hemisphere(cube Ao,cube Bo,
    float x1,float y1,float z1,
    float x2,float y2,float z2){
    A = Ao;
    B = Bo;
    r1 = new PVector(x1,y1,z1);
    r1.normalize();
    r2 = new PVector(x2,y2,z2);
    r2.normalize();
  }
  public void update(){
    PVector R1 = A.ori.toWorld(r1);
    PVector R2 = B.ori.toWorld(r2);
    float w = PVector.angleBetween(R1,R2);
    if(w>PI/2){
      PVector T = R1.cross(R2);
      T.normalize();
      PVector dAngVel = PVector.sub(B.angVel,A.angVel);
      float D = T.dot(dAngVel)*damping3;
      T.mult((w-PI/2)*spring3+D);
      A.torque.add(T);
      B.torque.sub(T);
    }
  }
}
class orient {
  PVector ui;
  PVector uj;
  PVector uk;
  orient() {
    ui = new PVector(1, 0, 0);
    uj = new PVector(0, 1, 0);
    uk = new PVector(0, 0, 1);
  }
  public void spin(PVector phi) {
    ui = handleturn(ui, phi);
    uj = handleturn(uj, phi);
    uk = handleturn(uk, phi);
  }
  public void restore() {
    uk = ui.cross(uj);
    uj = uk.cross(ui);
    ui.normalize();
    uj.normalize();
    uk.normalize();
  }
  public PVector toOri(PVector w){
    PVector o = new PVector();
    o.x = w.x * ui.x + w.y * ui.y + w.z * ui.z;
    o.y = w.x * uj.x + w.y * uj.y + w.z * uj.z;
    o.z = w.x * uk.x + w.y * uk.y + w.z * uk.z;
    return o;
  }
  public PVector toWorld(PVector o){
    PVector w = new PVector();
    w.x = ui.x * o.x + uj.x * o.y + uk.x * o.z;
    w.y = ui.y * o.x + uj.y * o.y + uk.y * o.z;
    w.z = ui.z * o.x + uj.z * o.y + uk.z * o.z;
    return w;
  }
  public PVector handleturn(PVector r, PVector w) {
    PVector handled = r;
    if ((w.mag() != 0) && (r.mag() != 0)) {
      PVector r2 = new PVector();
      r2.x = r.x;
      r2.y = r.y;
      r2.z = r.z;
      PVector w2 = new PVector();
      w2.x = w.x;
      w2.y = w.y;
      w2.z = w.z;
      r2.normalize();
      w2.normalize();
      if (r2.dot(w2) != 1) {
        handled = turn(r, w);
      }
    }
    return handled;
  }
  public PVector turn(PVector r, PVector w) {
    float dw = PI/8;
    int divisions;
    if (w.mag() > dw) {
      divisions = ceil(w.mag() / dw);
    } else {
      divisions = 1;
    }
    PVector uw = new PVector();
    uw.x = w.x;
    uw.y = w.y;
    uw.z = w.z;
    uw.normalize();
    PVector along = PVector.mult(uw, uw.dot(r));
    PVector rprime = PVector.sub(r, along);
    PVector phi = PVector.div(w, divisions);
    for (int i=1; i<=divisions; i++) {
      rprime = dturn(rprime, phi);
    }
    return PVector.add(rprime, along);
  }
  
  public PVector dturn(PVector rprime, PVector phi) {
    PVector o2 = PVector.div(phi, 2);
    PVector a = PVector.add(rprime, o2.cross(rprime));
    PVector ua = a;
    ua.normalize();
    PVector b = PVector.mult(PVector.sub(rprime,
                          PVector.mult(ua, ua.dot(rprime))), -1);
    return PVector.add(rprime, PVector.mult(b, 2));
  }
}
  static public void main(String args[]) {
    PApplet.main(new String[] { "--bgcolor=#FFFFFF", "Biomechanics" });
  }
}
