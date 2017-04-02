/*
The MIT License (MIT)
 
 Copyright (c) 2013 Sebatian Nilsson
 Modified by (c) 2017 Jeferson Lima
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

// If you want to debug the plotter without using a real serial port
int mockupValue = 0;
int mockupDirection = 10;
final String mockupDelimiter = ",";
final char mockupEnd = '\n';

String mockupSerialFunction() {
  mockupValue = (mockupValue + mockupDirection);
  if (mockupValue > 100)
    mockupDirection = -10;
  else if (mockupValue < -100)
    mockupDirection = 10;
  String r = "";
  for (int i = 0; i<6; i++) {
    switch (i) {
    case 0:
      r += mockupValue + mockupDelimiter;
      break;
    case 1:
      r += 100*cos(mockupValue*(2*3.14)/1000) + mockupDelimiter;
      break;
    case 2:
      r += mockupValue/4 + mockupDelimiter;
      break;
    case 3:
      r += mockupValue/8 + mockupDelimiter;
      break;
    case 4:
      r += mockupValue/16 + mockupDelimiter;
      break;
    case 5:
      r += mockupValue/32 + mockupDelimiter;
      break;
    }
  }
  r += mockupEnd;
  delay(10);
  return r;
}