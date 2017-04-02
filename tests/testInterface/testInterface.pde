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
 * Interface for test and configuration of Robozino
 * Use processing 3.1.X or above with the libraries listed bellow:
 *   > ControlP5
 *
 */

import controlP5.ControlP5;

/* P5 Controller */
ControlP5 cp5;

/* Number of lines in LineGraph */
final int numberOfLines = 2;

/* Time Interval */
final int timeInterval = 60;

/* Enable use of mockup serial */
final boolean mockupSerial = true;

/* Graph Line colors */
color graphColors[];

/* Graph of rotations of each motor */
Graph LineGraph;

/* Content of LineGraph */
float[][] lineGraphValues;
float[] lineGraphSampleNumbers;

/* Setup Interface */
void setup() {
  /* Create Interface */
  size(800, 600);
  noStroke();
  surface.setTitle("Robozino - Debug Interface");

  /* Create a new controller for cp5 lib */
  cp5 = new ControlP5(this);

  /* Configure Line Graph */

  /* Construct LineGraph */
  LineGraph = new Graph(230, 210, 470, 290, color (20, 20, 200));

  /* Labels of LineGraph */
  LineGraph.xLabel =" Time ";
  LineGraph.yLabel =" Steps ";
  LineGraph.Title =" Rotations of Each Motor ";  

  /* Num. of div. X axis / Max of X axis / Min of Y axis */
  LineGraph.xDiv = 6;
  LineGraph.xMax = 0;
  LineGraph.xMin = -60;

  /* Num. of div. of Y axis / Max of Y axis / Min of Y axis */
  LineGraph.yDiv = 10;
  LineGraph.yMax = 250;
  LineGraph.yMin = 0;

  /* Configurations of plot of LineGraph */

  /* Colors of the LineGraph */
  graphColors = new color[numberOfLines];
  graphColors[0] = color(131, 255, 20);
  graphColors[1] = color(232, 158, 12);

  /* Array of values of the LineGraph */
  lineGraphValues = new float[numberOfLines][timeInterval];

  /* I'm not sure what is that */
  lineGraphSampleNumbers = new float[timeInterval];

  /* Initialize matrix for line graph and samples */
  for (int i = 0; i < numberOfLines; i++) {
    for (int j = 0; j < timeInterval; j++) {
      /* Initialize lineGraphValues */
      lineGraphValues[i][j] = 0;
      
      /* Initialize sample number one time only */
      if(i == 0){
        lineGraphSampleNumbers[j] = j;
      }
    }
  }
}

/* Draw interface */
void draw() {

  /* Read serial and update values */
  //if (mockupSerial || serialPort.available() > 0) {
  if (mockupSerial) {
    String myString = "";
    if (!mockupSerial) {
      try {
        //serialPort.readBytesUntil('\r', inBuffer);
      }
      catch (Exception e) {
      }
      //myString = new String(inBuffer);
    } else {
      myString = mockupSerialFunction();
    }

    //println(myString);

    // split the string at delimiter (space)
    String[] nums = split(myString, ' ');

    //int numberOfInvisibleLineGraphs = 0;
    //for (int i = 0; i < 6; i++) {
    //  if (int(getPlotterConfigString("lgVisible"+(i+1))) == 0) {
    //    numberOfInvisibleLineGraphs++;
    //  }
    //}
    // update line graph
    for (int i = 0; i < nums.length; i++) {
      try {
        if (i < numberOfLines) {
          for (int k = 0; k < timeInterval - 1; k++) {
            lineGraphValues[i][k] = lineGraphValues[i][k+1];
          }

          lineGraphValues[i][lineGraphValues[i].length-1] = float(nums[i]);// * float(getPlotterConfigString("lgMultiplier"+(i+1)));
        }
      } 
      catch (Exception e) {
      }
    }
  }

  // draw the line graphs
  background(255); 
  LineGraph.DrawAxis();
  for (int i = 0; i < lineGraphValues.length; i++) {
    LineGraph.GraphColor = graphColors[i];
    if (true) { //(int(getPlotterConfigString("lgVisible"+(i+1))) == 1){
      LineGraph.LineGraph(lineGraphSampleNumbers, lineGraphValues[i]);
    }
  }
}