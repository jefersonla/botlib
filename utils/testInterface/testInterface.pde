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
import controlP5.ControlEvent;
import controlP5.Textfield;
import controlP5.Toggle;
import controlP5.Button;
import controlP5.ScrollableList;
import processing.serial.Serial;

/* Color Constants */
final color COLOR_WHITE = color(255);
final color COLOR_ORANGE_LIGHT = color(131, 255, 20);
final color COLOR_GREEN_LIGHT = color(232, 158, 12);

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

/* Display line check */
boolean displayLine[];

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
  LineGraph = new Graph(239, 225, 480, 290, color (20, 20, 200));

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
  graphColors[0] = COLOR_ORANGE_LIGHT;
  graphColors[1] = COLOR_GREEN_LIGHT;

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
      if (i == 0) {
        lineGraphSampleNumbers[j] = j;
      }
    }
  }

  /* Initialize visibility of LineGraph lines */
  displayLine = new boolean[numberOfLines];
  for (int i = 0; i < numberOfLines; i++) {
    displayLine[i] = true;
  }

  /* Resize Y axis limits input */
  int xAxisLineGraph = 180;
  int yAxisLineGraph = 195;
  int inputWidth = 40;
  cp5.addTextfield("lgMaxY")
    .setPosition(xAxisLineGraph, yAxisLineGraph)
    .setText("250")
    .setWidth(inputWidth)
    .setAutoClear(false);
  cp5.addTextfield("lgMinY")
    .setPosition(xAxisLineGraph, yAxisLineGraph + 290)
    .setText("0")
    .setWidth(inputWidth)
    .setAutoClear(false);

  /* Label for on/off button and change kp */
  int labelXPos = 10;
  int labelYPos = 160;
  cp5.addTextlabel("on/off")
    .setText("on/off")
    .setPosition(labelXPos + 25, labelYPos)
    .setColor(0);
  cp5.addTextlabel("pgain")
    .setText("P Gain")
    .setPosition(labelXPos + 75, labelYPos)
    .setColor(0);
  cp5.addTextlabel("lmark")
    .setText("L")
    .setPosition(labelXPos, labelYPos + 20)
    .setColor(0);
  cp5.addTextlabel("rmark")
    .setText("R")
    .setPosition(labelXPos, labelYPos + 49)
    .setColor(0);

  /* Enable graphic visualization of each steps counter */
  int gainXPos = 30;
  int gainYPos = 174;
  cp5.addToggle("lgStepsL")
    .setPosition(gainXPos, gainYPos)
    .setValue(true)
    .setMode(ControlP5.SWITCH)
    .setColorActive(graphColors[0]);
  cp5.addToggle("lgStepsR")
    .setPosition(gainXPos, gainYPos + 30)
    .setValue(true)
    .setMode(ControlP5.SWITCH)
    .setColorActive(graphColors[1]);

  /* Input for P Gain */
  cp5.addTextfield("pGainL")
    .setPosition(gainXPos + 52, gainYPos)
    .setText("1.0")
    .setWidth(inputWidth)
    .setAutoClear(false);
  /* Input for P Gain */
  cp5.addTextfield("pGainR")
    .setPosition(gainXPos + 52, gainYPos + 30)
    .setText("1.0")
    .setWidth(inputWidth)
    .setAutoClear(false);

  /* Display serial avaialable dropdown list and label*/
  cp5.addTextlabel("serialListLabel")
    .setText("Serial List")
    .setPosition(673, 115)
    .setColor(0);
  cp5.addScrollableList("Serial List")
    .setPosition(673, 130)
    .setSize(100, 200)
    .setBarHeight(20)
    .setItemHeight(20)
    .addItems(Serial.list())
    .setOpen(false)
    .setType(ScrollableList.DROPDOWN);
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

  /* Print Background with white color */
  background(COLOR_WHITE);

  /* Print the LineGraph */
  LineGraph.DrawAxis();
  for (int i = 0; i < numberOfLines; i++) {
    /* Check if the line is configured to be displayed */
    if (displayLine[i]) {
      /* Chose color of the actual line */
      LineGraph.GraphColor = graphColors[i];
      /* Update the graph with the time */
      LineGraph.LineGraph(lineGraphSampleNumbers, lineGraphValues[i]);
    }
  }
}

/* Check which event has occured */
void controlEvent(ControlEvent theEvent) {

  /* Get name of interface which perform the event */
  String parameter = theEvent.getName();

  /* If the event was triggered from a text field */
  if (theEvent.isAssignableFrom(Textfield.class)) {

    /* Get value of textfield */
    String value = theEvent.getStringValue();

    /* Now change parameters accordily */
    switch(parameter) {
    case "lgMaxY": 
      LineGraph.yMax = float(value);
      infoMsg("UPDATING MAX LIMIT OF LINE GRAPH");
      break;
    case "lgMinY":
      LineGraph.yMin = float(value);
      infoMsg("UPDATING MIN LIMIT OF LINE GRAPH");
      break;
    default:
      errorMsg("UNKNOW TEXTFIELD EVENT - " + parameter + " - " + value);
    }
  }
  /* Was triggered from a switch button */
  else if (theEvent.isAssignableFrom(Toggle.class)) {

    /* Get value of the toggle switch button */
    boolean value = theEvent.getValue() > 0 ? true : false;

    /* Check which operations need to be perfomed */
    switch(parameter) {
    case "lgStepsL":
      displayLine[0] = value;
      break;
    case "lgStepsR":
      displayLine[1] = value;
      break;
    default:
      errorMsg("UNKNOW SWITCH EVENT - " + parameter + " - " + value);
    }
  }
}

/* Debug Messages */

/* Info Message */
void infoMsg(String msg) {
  System.out.println("[INFO] " + msg);
}

/* Warning Message */
void warningMsg(String msg) {
  System.err.println("[WARNING] " + msg);
}

/* Error Message */
void errorMsg(String msg) {
  System.err.println("[ERROR] " + msg);
}