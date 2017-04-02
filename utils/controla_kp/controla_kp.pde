/*
    O codigo do Arduino deve ser algo como:
    
    void loop() {
      if (Serial.available()) { // If data is available to read,
        val = Serial.read(); // read it and store it in val
      }
      if (val == '1') { // If 1 was received
         kp_direita--;
      if (val == '2') { // If 1 was received
         kp_direita++;
      if (val == '3') { // If 1 was received
         kp_esquerda--;
      if (val == '4') { // If 1 was received
         kp_esquerda++;
      }
      delay(10); // Wait 10 milliseconds for next reading
    }

*/

import controlP5.*;
import processing.serial.*;

Serial myPort;  // Create object from Serial class
ControlP5 botao;

color verde = color(0, 255, 0);
color vermelho = color(255, 0, 0);
int myColor = color(0);

PFont f;

void setup()
{
  frameRate(240);
  size( 640, 480 );
  smooth();
  // Step 3: Create Font
  f = createFont("Arial", 16);
  
  // I know that the first port in the serial list on my mac
  // is Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 9600);
  
 
  botao = new ControlP5(this);
  // replace the default controlP5 button with an image.
  // button.setImages(defaultImage, rolloverImage, pressedImage);
  // use button.updateSize() to adjust the size of the button and 
  // resize to the dimensions of the defaultImage
  
  botao.addButton("MENOS_DIR")
     .setPosition(60,270);
     
  botao.addButton("MAIS_DIR")
     .setPosition(510,270);
  
  botao.addButton("MENOS_ESQ")
     .setPosition(60,390);
  
  botao.addButton("MAIS_ESQ")
     .setPosition(510,390);
     //desnecessario
     /*.setImages(loadImage("Arrow-Left.png"), loadImage("Arrow-Right.png"), loadImage("Refresh.png"))
     .updateSize();*/

}

void draw()
{
  background( 255 );
  fill( 0 ); //128, 0, 0
  textFont( f, 30 );
  textAlign( CENTER );
  text( "Olá, Eu sou a Interface do Robozino", 320, 60 );
  textFont( f, 24 );
  text("Essa são as constantes Kps.", 320, 90);
  stroke( 0 );
  strokeWeight( 1 );
  line( 20, 94, 620, 94 );
  line( 20, 96, 620, 96 );
  
  fill( 0 );
  rect(200,240,240,60);
  fill(255);
  textFont( f, 30 );
  textAlign( CENTER );
  text( "Kp Direita", 320, 280 );
  fill( 0 );
  rect(200,360,240,60);
  fill(255);
  textFont( f, 30 );
  textAlign( CENTER );
  text( "Kp Esquerda", 320, 400 );
  
  /*
  // Bright red
  fill(vermelho);
  ellipse(100,270,60,60);
  fill(0);
  textFont( f, 30 );
  textAlign( CENTER );
  text("-", 100,280);
  fill(verde);
  ellipse(540,270,60,60);
  fill(0);
  textFont( f, 30 );
  textAlign( CENTER );
  text( "+", 540, 280 );
  
  
  // Bright red
  fill(vermelho);
  ellipse(100,390,60,60);
  fill(0);
  textFont( f, 30 );
  textAlign( CENTER );
  text( "-", 100, 400 );
  fill(verde);
  ellipse(540,390,60,60);
  fill(0);
  textFont( f, 30 );
  textAlign( CENTER );
  text( "+", 540, 400 );
  */
}

public void controlEvent(ControlEvent theEvent) {
  println(theEvent.getController().getName());
  
}

public void MENOS_DIR(int theValue) {
    println("Decrementa Direita: "+theValue);
    myPort.write('1');         //send a 1
    myColor = color(128);
}

public void MAIS_DIR(int theValue) {
    println("Incrementa Direita: "+theValue);
    myPort.write('2');         //send a 1
    myColor = color(128);
}

public void MENOS_ESQ(int theValue) {
    println("Decrementa Esquerda: "+theValue);
    myPort.write('3');         //send a 1
    myColor = color(128);
}

public void MAIS_ESQ(int theValue) {
    println("Incrementa Esquerda: "+theValue);
    myPort.write('4');         //send a 1
    myColor = color(128);
}
