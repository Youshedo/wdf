ArrayList gravatons;
float count = 1000;
//float grav = 67.225624;
//float k = 992.397463;
///float grav = 6.283473;
float grav = 9.87654321;
float k = 1.1000001;
//float mass = 1.5366822E-8;
float mass = 0.0020000117;
float h = 15%100;
//float dran = 0.7000007;
float dran = 0.20000067;
boolean Dmatter = true;

void setup() {
  size(1000,900);
  //size(screen.width,screen.height,P3D);
  //fill(255);
  gravatons = new ArrayList();
  for(int i=0; i<count;i++) {
    gravatons.add( new gravaton() );
  }
  smooth();
}

class gravaton {
  PVector x, y, f;
  gravaton() {
    x = new PVector( width/2 + random(3) ,height/2 + random(3) );
    //x = new PVector(random(width),random(height));
    y = new PVector();
    f = new PVector();
  }

  void update() {
    y.add( f );
    f = new PVector();
    x.add( y );
  }
}

void draw() {
 background( 0 );
  for(int i=1;i<gravatons.size();i++) {
    gravaton A = (gravaton)gravatons.get( i );
    for(int j=0;j<i/2;j++) {
      gravaton B = (gravaton)gravatons.get( j );
      PVector dr = PVector.sub( B.x, A.x );
      if(sqrt( dr.x * dr.x + dr.y * dr.y ) < h * grav ) {
        set(int( A.x.x ), int( A.x.y ), color(255,0,0));
        set(int( B.x.x ), int( B.x.y ), color(0,255,0));
        float pt1 = (( h - sqrt( dr.x * dr.x + dr.y * dr.y ))* mass );
        dr.normalize();
        float pt2 = sqrt( dr.x * dr.x + dr.y * dr.y ) * sqrt( dr.x * dr.x + dr.y * dr.y ); 
        float frc = ( pt1 - pt2 );
        dr.mult( frc );
        A.y.sub( dr );
        B.y.add( dr );
      }
    }
  }


  for(int i=0;i<gravatons.size();i++) {
    gravaton A = (gravaton) gravatons.get( i );
    PVector mouseV = new PVector( mouseX, mouseY );
    PVector pmouseV = new PVector( pmouseX, pmouseY );
    if(mousePressed) {
      PVector dr = PVector.sub( A.x, mouseV );
      float pushrad = pow( grav, 2.2 );
      if(sqrt( dr.x * dr.x + dr.y * dr.y ) < pushrad ) { 
        dr.normalize();
        A.f.add(PVector.mult( dr, -10 ));
        A.y.add(PVector.mult(PVector.sub(
        PVector.sub(mouseV,pmouseV), A.y ), 0.01 ));
      }
    }
  }
  for(int i=0;i<gravatons.size();i++) {
    gravaton A = (gravaton) gravatons.get(i);

    if(Dmatter) { 
      A.y.mult( dran );
    }
    if (A.x.x < 0) { A.x.x = width-1; }
    else if (A.x.x > width-1) { A.x.x = 0; }
    if (A.x.y < 0) { A.x.y = height-1; }
    else if (A.x.y > height-1) { A.x.y = 0; }
    A.update();
  }
  if ( 2 >= frameRate ) {
    setup();
  }

  if( this.frameCount%2 == 100 ) {
    gravatons.add( new gravaton() );
  } 
  fill( 255 );
  text("(((Electron&Atoms)))  framerate =" + frameRate, 10, 15);
  text("grav = " + grav + ", particles = " + count,     10, 30);
  text("k = " + k,                                      10, 45);
  text("mass = " + mass,                                10, 60);
  text("h = " + h,                                      10, 75);
  text("dran = " + dran,                                10, 90);
  text("A = " + (gravaton)gravatons.get(int(random(1))),10,105);
 
}


void keyPressed() {
  if (key == ' ') {
    setup();
    background(0);
  }
  if (key == 'q') { grav += 0.1; }
  if (key == 'a') { grav -= 0.1; }
  if (key == 'w') { k += 0.1; }
  if (key == 's') { k -= 0.1; }
  if (key == 'e') { mass += 0.002; }
  if (key == 'd') { mass -= 0.002; }
  if (key == 'r') { dran += 0.1; }
  if (key == 'f') { dran += -0.1; }
}
