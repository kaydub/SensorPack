/*
  VoltBarn Sensor Pack monitor
  language: processing
  
  Note that this requires an XBee attached as a serial device to a USB port. You have to run the program, find out what port it thinks your
  XBee is on, then change the serialPort variable to match.
 
 The program receives data from one or more SensorPacks, and from that data:
 - Creates a primitive graph of the data, and saves the graph periodically: wgraph.png
 - Writes every packet recieved with a timestamp to: SensorPack.txt
 - Writes the latest update to: sensorReading.txt
 - Rebroadcasts the packet, which you could use to create a web of SensorPacks and displays with a range greater than any individual SensorPack
 This can be used by other programs to remotely display the weather information
 - Optionally Updates Weather Underground
 - Optionally calls a web service interface
 
 Need to customize this code to provide your own stationID(s)/password for Weather Underground.
 
 This was really just a quick hack, but it got me up and running quickly. I've actually used the software for years now to keep my 
 weather underground updated, and it's a easy way to make sure you've got things running correctly.
 
 created 1 May. 2010
 last modified 28 July. 2012
 by Ken Wallich
 http://voltbarn.com
 */

String sensorID = "VoltBarn Anonymous";

// This will update the weather for multiple Weather Underground accounts. Since I wanted to deploy the same code in multiple locations, I simply put in
// my sensorPack ID's, and then the associate login info for each account. Should be pretty obvious how that works...
// If you don't use weather underground, just leave as is and the code will never match, unless you somehow named your sensorPacks with these ID's

String sensorPack1 = "yourIDHereOne";
String sensorPack2 = "yourIDHereTwo";
String wundString1 = "ID=Kxxxxxx00&PASSWORD=myPass";
String wundString2 = "ID=Kxxxxxx01&PASSWORD=myPass";

// I set up a webService to grab all the information. It just takes a sensorPack data packet, leaving the basic code
// in case you want to do the same
boolean updateSpack = false; 

int serialPort = 2; // Change this based on reported serial port, need to automate this!

// create a font with the second font available to the system:
PFont myFont = createFont(PFont.list()[2], 14);
PFont smallFont = createFont(PFont.list()[2], 10);

import processing.serial.*;

Serial myPort;           // the serial port you're using
String portnum;          // name of the serial port
String outString = "";   // the string being sent out the serial port
String inString = "";    // ththee string coming in from the serial port
String theString = "";   // just the current line we're working with
String stationString = "";
float temp = 50;
float humid = 50;
int ip = 295;
float pf = 29.5;
float wind = 0;
float windGust = 0;
int direction;
float rain;
float rainTotal;
int receivedLines = 0;   // how many lines have been received in the serial port
int bufferedLines = 25;  // number of incoming lines to keep
PrintWriter output;
int nupdates = 0;
int refreshTime;
float dpf = 0, tf= 0, hf = 0;

void setup() {
  String timeS;

  size(600, 200);        // window size
  background(100);

  // create a font with the second font available to the system:
  //PFont myFont = createFont(PFont.list()[2], 14);
  textFont(myFont);

  output = createWriter("SensorPack.txt"); 

  // list all the serial ports:
  println(Serial.list());

  timeS = "Sensor Pack Output started: ";
  timeS += str(month());
  timeS += '/';
  timeS += str(day());
  timeS += '/';
  timeS += str(year());
  timeS += "  ";
  timeS += str(hour());
  timeS += ':';
  timeS += str(minute());
  timeS += '.';
  timeS += str(second());
  output.println(timeS);


  // based on the list of serial ports printed from the
  //previous command, change the 0 to your port's number:
  portnum = Serial.list()[serialPort];

  fill(0);
  textFont(myFont);
  text("Serial port: " + portnum, 0, 20);

  // initialize the serial port:
  myPort = new Serial(this, portnum, 9600);
  //  myPort.buffer(10);
  output.flush(); // Writes the remaining data to the file
  refreshTime = 0;

}

void bar(int x,int y, int h, int bmin, int bmax, int val, boolean blu, String l) {
  int w=20;

  fill(0);
  textFont(smallFont);
  text(str(bmax), x+w+10, y+10); // Bar max indicator
  text(str(bmin), x+w+10, y+h); // Bar min indicator
  text(l, x, y+h+10);
  textFont(myFont);
  //text(str(val), x+w+10, y+(h-(h/bmax)*val)+10);
  float fval = val, fbmin=bmin, fbmax=bmax, fh = h; // need to use floats for bsize
  int bsize = int((fval-fbmin)/(fbmax-fbmin)*h); // Absolute size of the bar value
  int lp = y+h-bsize+10;
  text(str(val), x+w+10, lp);
  rectMode(CORNER);
  fill(255);
  rect(x,y,w,h);
  fill(255,30,30);
  if (blu && (val <= 32))
    fill(30,30,255);
  rect(x,lp-10,w,bsize);

}

void draw() {
  /* code to time to weather station */
  int dtemp, dhumid, dpres;
  int hr, mn;
  String line;

  if (refreshTime < millis()) {

    refreshTime = millis() + 30000;

    // Send time to weather station
    hr = hour();
    mn = minute();
    myPort.write(":c:"); // Tag this as a time packet
    if (hr < 10)
      myPort.write("0");
    myPort.write(str(hr));
    if (mn < 10)
      myPort.write("0");
    myPort.write(str(mn));
    myPort.write(":");

  } // Refresh time
}

void reBroadcast(String tag, String value) {
  // Rebroadcast certain fields to increase likelyhood that weather displays get the info
  myPort.write(":" + tag + ":" + value + ":");
}

// this method runs when bytes show up in the serial port:
void serialEvent(Serial myPort) {
  int timeNow = millis();
  UTCtime dt;
  String timeS, urlString, twitString;
  int s = second();  // Values from 0 - 59
  int m = minute();  // Values from 0 - 59
  int h = hour();    // Values from 0 - 23
  boolean updateWund = false;
  int dpi, x;

  dt = new UTCtimUTe();
prin
  // read the next byte from the serial port:
  int inByte = myPort.read();
  // Check value, first byte is sometimes garbage
  /*
  if (theString.length() == 0)
   println(str(inByte));
   */
  // add it to  inStrng:

  // Make sure we didn't get a blank newline, or we send out bogus data
  if ((inByte == '\r') && (theString.length() > 1)) { 
    String[] st = splitTokens(theString, ": ");

    print(theString.length());
    print(":");
    println(theString);
    nupdates++;

    background(100);

    for (int i=0; i < st.length; i++) {
      if (st[i].equals("t")) { //SHT15 keeps breaking, using SCP1000 temp
        reBroadcast(st[i], st[i+1]);
        tf = float(st[i+1]);
        temp = int(tf);
        bar(20,75,100,0,100,int(temp),true, "Temp");
      }
      if (st[i].equals("h")) {
        reBroadcast(st[i], st[i+1]);
        hf = float(st[i+1]);
        humid = int(st[i+1]);
        bar(120,75,100, 0,100,int(humid),false, "Humidity");
      }
      if (st[i].equals("p")) {
        reBroadcast(st[i], st[i+1]);
        pf = float(st[i+1]);
        ip = int(pf * 100);       
        bar(220,75,100,2800,3000,ip,false, "Pressure");
      }
      if (st[i].equals("w")) {
        wind = int(st[i+1]);
      }
      if (st[i].equals("g")) {
        windGust = float(st[i+1]);
      }
      if (st[i].equals("d")) {
        direction = int(st[i+1]);
      }
      if (st[i].equals("r")) {
        rain = float(st[i+1]);
      }
      if (st[i].equals("R")) {
        rainTotal = float(st[i+1]);
      }
      if (st[i].equals("a") || st[i].equals("id")) {
        sensorID = st[i+1];
        reBroadcast(st[i], st[i+1]);
      }

      //println(st[i]);
    }

    /* Calculate temp and dewpoint */
    float a = 17.271;
    float b = 237.7;
    float tc = (temp-32) * (5.0/9.0);
    float y = (a*tc)/(b+tc) + log(humid/100.0);
    float dpc = (b*y)/(a-y);
    dpf = (dpc * (9/5)) + 32;

    if (wind > 0) {
      bar(320, 75, 100, 0, int(windGust), int(wind), false, "Windspeed");
    }
    if (rain > 0) {
      bar(420, 75, 100, 0, int(rainTotal), int(rain), false, "Rainfall");
    }

    timeS = sensorID;
    timeS += " update: ";
    timeS += str(month());
    timeS += '/';
    timeS += str(day());
    timeS += '/';
    timeS += str(year());
    timeS += " ";
    timeS += str(hour());
    timeS += ':';
    timeS += str(minute());
    timeS += '.';
    timeS += str(second());
    timeS += " updates: ";
    timeS += str(nupdates);
    fill(255);
    textFont(myFont);
    text(timeS, 0, 40);

    save("wgraph.png");

    PrintWriter intOutput = createWriter("sensorReading.txt"); 

    intOutput.println(theString);
    intOutput.flush();
    intOutput.close();
    
    //Update Spack
    if(updateSpack) {
      String[] SpackS = { 
        "/usr/bin/curl", "-X", "POST",  
        "--data", "post[sid]=" + sensorID,
        "--data", "post[datetime(1i)]=" + str(dt.Year),
        "--data", "post[datetime(2i)]=" + str(dt.Month),
        "--data", "post[datetime(3i)]=" + str(dt.Day),
        "--data", "post[datetime(4i)]=" + str(dt.Hour),
        "--data", "post[datetime(5i)]=" + str(dt.Minute),
        "--data", "post[packet]=" + theString,
        "--data", "commit=Create",
        "http://host.name/posts.xml"   
      };

      exec(SpackS);
    }
    // updateSpack

    theString += ' ';
    theString +=timeS;
    // if the byte is a carriage return, print
    // a newline and carriage return:
    theString += '\n';
    inString += theString;

    output.println(theString);
    output.flush();
    stationString = theString;
    theString = "";

    /* send to Weather Underground */

    /* Send the info to Weather Underground */
    /* See: http://wiki.wunderground.com/index.php/PWS_-_Upload_Protocol for url strings to use */

    urlString = "http://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?";

    if(sensorID.equals(sensorPack1)) {
      urlString += wundString1;
      updateWund = true;
    } 
    else if (sensorID.equals(sensorPack2)) {
      urlString += wundString2;
      updateWund = true;
    } 
    else {
      updateWund = false;
    }

    urlString += "&dateutc=";
    urlString += str(dt.Year);
    urlString += '-';
    urlString += str(dt.Month);
    urlString += '-';
    urlString += str(dt.Day);
    urlString += ' ';
    urlString += str(dt.Hour);
    urlString += ':';
    urlString += str(dt.Minute);
    urlString += ':';
    urlString += str(dt.Second);
    if (humid > 0) { // SHT 15 having problems
      urlString += "&humidity=";
      urlString += str(humid);
    }
    urlString += "&tempf=";
    urlString += str(tf);
    urlString += "&baromin=";
    urlString += str(pf); 
    urlString += "&dewptf=";
    urlString += str(dpf);
    urlString += "&windspeedmph=";
    urlString += str(wind);
    urlString += "&windgustmph=";
    urlString += str(windGust);
    urlString += "&winddir=";
    urlString += str(direction);

    if (updateWund) {
      try {
        URL url = new URL(urlString);
        url.openConnection();
        BufferedReader in = new BufferedReader(new InputStreamReader(url.openStream()));
        String inputLine;
        StringBuilder builder = new StringBuilder(); 
        while ((inputLine = in.readLine()) != null)
          builder.append(inputLine); 
        in.close();
        print(builder.toString());

      } 
      catch (MalformedURLException e) {
        println("MalformedURLException");
        println(e);
      } 
      catch (IOException e) {               // openConnection() failed
        println("IOException");
        println(e);
      } 
      //debug println(urlString);
    } // updateWund

    urlString = "";  // Clear URL string for next pass
  } // if we got a \r, and theString > 1
  else {
    if (inByte == '\r') // We just got a null string, erase
      theString = "";
    else // We got a new byte, but not an end of line
    theString += char(inByte);
  }
}

String snarfNum() {

  String theString = "";
  int inByte = myPort.read();

  if (inByte == ':') {
    while(inByte != ' ') {
      inByte = myPort.read();
      theString += inByte;
    }  
    return(theString);
  }  
  else return("");
}

// deletes the top line of inString so that it all fits on the screen:
void deleteFirstLine() {
  // find the first newline:
  int firstChar = inString.indexOf('\n');
  // delete it:
  inString= inString.substring(firstChar+1);
}

class UTCtime {
  int Hour;
  int Minute;
  int Second;
  int Day;
  int Month;
  int Year;

  UTCtime () {
    Hour = hour();
    Minute = minute();
    Second = second();
    Day = day();
    Month = month();
    Year = year();
    int dst = 1;
    int o = 8-dst; // PT, added dst just 'cause.

    if (Hour + o < 23) {
      Hour = Hour + o;  // easy case
    } 
    else {
      Hour = (Hour + o) % 24;
      if (Month == 2) {
        if (Day == 28) {
          Month++; 
          Day=1;
        } 
        else {
          Day++;
        }
      } 
      else if ((Month==4 || Month==6 || Month==9 || Month==11)) {
        if (Day == 30) {
          Month++; 
          Day=1;
        } 
        else {
          Day++;
        }
      } 
      else if (Day == 31) {
        Month++; 
        Day=1;
      } 
      else {
        Day++; 
      }
    }
  }
}



