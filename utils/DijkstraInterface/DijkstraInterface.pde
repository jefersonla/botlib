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

import processing.serial.*;

/* Create object from Serial class */
Serial myPort;        
/* Data received from the serial port */
String val;           
/* Variable to store (int)val */
int valor;            
/* Variable to store last square visited */
int last;             
/* Variable to define first interaction, set origin position */
boolean first = true; 


/* Color Constants */
final color preto = color(0);
final color verde = color(0, 158, 0);
final color azul = color(13, 255, 243);
final color roxo = color(200, 46, 232);
final color vermelho = color(255, 0, 0);

/* Creating the Grid */
final int N = 3;
final int M = 5;
final short NUM = N*M;

/* Create an array to store widths and lenghts */
PVector[] vectors = new PVector[NUM];

final int largura = 640 / N;
final int altura = 480 / M;
int iniciox = 0;
int inicioy = 0;
int z = 0;
int counter = 0;

/* Serial Speed */
final int SERIAL_SPEED = 9600;

/* Create a instance of type Font, later it will receive the Arial font, with 16 px */
PFont f;

void setup() {

  /* Create Interface */
  size(640, 480);
  noStroke();
  surface.setTitle("Robozino - Dijkstra Interface");

  f = createFont("Arial", 16);

  /*I know that the first port in the serial list on my mac
   is Serial.list()[0]. On Windows machines, this generally opens COM1.
   Open whatever port is the one you're using.*/

  /* Change the 0 to a 1 or 2 etc. to match your port */
  println((Object[])Serial.list());
  String portName = Serial.list()[0]; 
  
  myPort = new Serial(this, portName, SERIAL_SPEED);

  /* Initialize all the grid as unvisited */
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < M; j++) {
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
  /* If data is available */
  if ( myPort.available() > 0) {  
    val = myPort.readStringUntil('\n');
    valor = Integer.parseInt(val);
    /* If it is the first interaction, so print as oringin*/
    if (first) {
      fill( azul );
      rect(vectors[valor].x, vectors[valor].y, largura, altura);
      vectors[valor].z = 1;
      last = valor;
      first = false;
    }
    /* If valor is equal the total of square's numbers, so print the last square visited as final*/
    else if (valor == NUM) {
      fill( roxo );
      rect(vectors[last].x, vectors[last].y, largura, altura);
    }
    /* If valor is bigger than the total of square's numbers, so print the last square as barrier */
    else if (valor > NUM) {
      fill( preto );
      rect(vectors[last].x, vectors[last].y, largura, altura);
    }
    /* Print the square as visited */
    else {
      if (vectors[valor].z == 0) {
        fill( verde );
        rect(vectors[valor].x, vectors[valor].y, largura, altura);
        last = valor;
      }
    }
  }
}