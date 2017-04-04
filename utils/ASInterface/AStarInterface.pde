/**
 * MIT License
 * 
 * Copyright (c) 2017 Jeferson Lima & Andressa Andrade
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
/**
 *
 * Interface for visualization A Star Path
 * Use processing 3.1.X or above with the libraries listed bellow:
 *   > ControlP5
 *
 */

import processing.serial.*;

Serial myPort;        // Create object from Serial class
String val;           // Data received from the serial port
int valor;            // Variable to store (int)val
int last;             // Variable to store last square printed
boolean first = true; // Variable to define first interaction

import controlP5.*;

ControlP5 cp5;

/* Color Constants */
final color preto = color(0);
final color verde = color(0, 158, 0);
final color azul = color(13, 255, 243);
final color roxo = color(200, 46, 232);
final color vermelho = color(255, 0, 0);

/* Creating the Grid */
final int N = 3;
final short NUM = N*N;
// Create an array to store vectors
PVector[] vectors = new PVector[NUM];

final int largura = 640 / N;
final int altura = 480 / N;
int iniciox = 0;
int inicioy = 0;
int z = 0;
int counter = 0;

/* Serial Speed */
final int SERIAL_SPEED = 9600;

/* Set Interval */
final int interval = 500;

PFont f;

void setup() {

  /* Create Interface */
  size(640, 480);
  noStroke();
  surface.setTitle("Robozino - AStar Interface");

  f = createFont("Arial", 16);

  // I know that the first port in the serial list on my mac
  // is Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.

  String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, SERIAL_SPEED);

  /* Create a new controller for cp5 lib */
  cp5 = new ControlP5(this);

  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      fill( vermelho );
      rect(iniciox, inicioy, largura, altura);
      vectors[counter] = new PVector(iniciox, inicioy, z);
      iniciox += largura;
      counter++;
    }
    inicioy += altura;
    iniciox = 0;
  }
}

void draw() {
  if ( myPort.available() > 0) {  // If data is available,
    valor = myPort.read(); 
    if (first) {
      fill( azul );
      rect(vectors[valor].x, vectors[valor].y, largura, altura);
      vectors[valor].z = 1;
      last = valor;
      first = false; 
    }
    else if(valor == NUM){
      fill( roxo );
      rect(vectors[last].x, vectors[last].y, largura, altura);
    }
    else if (valor > NUM) {
      fill( preto );
      rect(vectors[last].x, vectors[last].y, largura, altura);
    }
    else {
      if(vectors[valor].z == 0){
        fill( verde );
        rect(vectors[valor].x, vectors[valor].y, largura, altura);
        last = valor;
      }
    }
  }
}
