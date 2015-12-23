/**
 * Serial Data Dumper 
 * 
 * Opens a serial connection and communicates with Maple Mini analog recorder.
 * Features:
 * - setup serial port parameters
 * - menu to choose one of the recording options:
 *   - setup recording parameters (recording time, sampling frequency, channels per sequence)
 *   - start recording
 *   - dump recoreded data
 *
 */
import processing.serial.*;

Serial myPort;      // The serial port
String portName = "";
int bRate = 0;
int whichKey = 0; int myKey = 0; // Variable to hold keystoke value
int[] keys = new int[5];  // max 5 key possibilities
int inByte = -1;    // Incoming serial data
int serial_ok = -1;
int rec_ok = -1;
byte[] inBuffer = new byte[512];
long index = 0;
int cnt = 0;
long tim;
long TIMEOUT = 100;  // millis
int rec_time = 1000, sampling_freq = 26, channels_per_seq = 4;
boolean bin_rec;
long bin_len;
byte[] data;
byte screen = 0;
String[] disp = {"","","","","","","","","","","","","","","",""};
/*********************************************************************/
void setup() {
  size(600, 400);
  background(0);
  // create a font with the third font available to the system:
  //PFont myFont = createFont(PFont.list()[8], 14);
  PFont myFont = createFont("Courrier new", 16);
  textFont(myFont);
  // init serial port
  ListSerial();
}
/********************************************************************/
void ListSerial()
{
  disp_line = 0;
  DisplayClearLines();
  int len = Serial.list().length;
  if ( len==0 ) {
    DisplayAddLine("No port available, cannot send or receive!",0,0);
    return;
  } else {
    DisplayAddLine("Select serial port:",0,0);
    DisplayAddLine("------------------------",-1,0);
    for ( byte i=0; i<len; i++)  DisplayAddLine("["+i+"]"+" : "+Serial.list()[i],-1,0);
    serial_ok = 0;
  }
  ShowScreen();
}
/********************************************************************/
void SetupSerial()
{
  switch (serial_ok) {
    case 0:  // show selected serial port and Baudrate options
      if (whichKey>='0' && whichKey<'0'+Serial.list().length) { // check valid selection
        // Open selected port
        portName = Serial.list()[whichKey-'0'];
        DisplayAddLine("- selected serial port: "+portName,0,0);
        String[] scr1 = {"","Select one configuration (bitrate, data bits, parity, stop bits):",
                          "-------------------------------------------------------------------------------",
                          "[0] : 115200, 8, N, 1","[1] : 250000, 8, N, 1","[2] : 500000, 8, N, 1"};
        DisplayAddLines(scr1,-1,0);
        serial_ok ++;  // goto next parameter selection
      }
      break;
    case 1:  // show selected Baudrate and data bits option
        switch (whichKey) {
          case '0':  bRate = 115200;  break;
          case '1':  bRate = 250000;  break;
          case '2':  bRate = 500000;  break;
          default: break;
        }
        if ( bRate>0 ) {
          String[] scr2 = {"- selected Baudrate: "+bRate,"","Open port "+portName+", "+bRate+", 8, N, 1 ? [ y / n ]"};
          DisplayAddLines(scr2,1,0);
          DisplayClearLines();
          serial_ok ++;  // goto next parameter selection
        }
        break;
    case 2:  // show selected data bits and parity options
        switch (whichKey) {
          case 'y':  case 'Y':
            myPort = new Serial(this, portName, bRate, 'N', 8, 1);  // Maple Mini analog recorder
            //myPort = new Serial(this, portName, 115200, 'E', 8, 2);  // EnergyCam
            if ( myPort!=null ) {
              DisplayAddLine("Serial port "+portName+", "+bRate+", 8, 'N', 1 opened successfully.",0,0);
              DisplayClearLines();
              ListRecordingOptions();
              serial_ok ++;  // goto next parameter selection
            } else {
              DisplayAddLine("Opening port "+portName+", "+bRate+", 8, 'N', 1 ... failed!",-1,-1);
              DisplayAddLine("Check serial port and try again (press 'n')",-1,0);
            }
            break;
          case 'n':  case 'N':
              ListSerial(); break;
          default: break;
        }
        break;
    default:  break;
  }
}
/********************************************************************/
void ParseBinaryData(int len)
{
  if ( data==null ) {
    data = new byte[len];
    arrayCopy(inBuffer, data);
    index = 0;
  } else data = concat(data, inBuffer);
    // add new bytes to the data array
    index += len;
    DisplayAddLine("index = "+index,-1,-1);
}
/********************************************************************/
void ParseInput(String inStr)
{
  String[] q = splitTokens(inStr, "\r\n");
  for (byte i=0; i<q.length;i++)  DisplayAddLine(q[i], -1,0);  // display receieved serial data
  if ( inStr.indexOf(">>>")>0 ) {
    bin_rec = true;
    DisplayAddLine("switched to binary reception", -1,0);
    // parse the binary length
    String[] m = match(inStr, "binary_length:\\s*(\\d*)");
    if ( m!=null ) {
      bin_len = parseInt(m[1]);
      DisplayAddLine("bin_len = "+bin_len,-1,0);
      tim = millis();  // start time-out
    }
    index = 0;
  }
}
/********************************************************************/
int disp_line = 0;
/********************************************************************/
void DisplayAddLine(String str, int line, int offset)
{
  int dLen = disp.length;
  int scroll = (disp_line+1+offset)-dLen;
  if ( scroll>0) {disp_line = dLen-1; println("scroll is: "+scroll);}
  while ( line==-1 && scroll>0 ) {  // have to scroll?
    for (int i=0; i<dLen-1; i++) disp[i] = disp[i+1];  // scroll all lines one up
    scroll--;
  }
  if ( line==-1 ) {  // add to last available line
    disp[disp_line+=offset] = str;
    if ((++disp_line)>dLen) disp_line = dLen;  // plausability check for line
  } else {
    disp[line] = str;
    disp_line = line+1;
  }
  //println("disp_line = "+disp_line);  // debug
}
/********************************************************************/
void DisplayAddLines(String[] scr, int line, int offset)
{
  DisplayAddLine(scr[0],line,offset);  // insert first line
  for ( int i=1; i<scr.length; i++) DisplayAddLine(scr[i],-1,0);
}
/********************************************************************/
void DisplayClearLines()
{
  for ( int i=disp_line; i<disp.length; i++) disp[i]="";
}
/********************************************************************/
void CheckSerial()
{
  if ( serial_ok<6 ) return;  // do it only if serial setup was successfull.

  int recs = myPort.available();
  if ( recs>0 ) {
    int inBytes = 0;
    if ( bin_rec ) {  // binary reception
      inBytes = myPort.readBytes(inBuffer);
      if (inBytes<0)  println("buffer overflow!");
      ParseBinaryData(inBytes);
      tim = millis();
    } else {  // string reception
      DisplayAddLine("- received "+recs+" bytes",-1,0);
      String inStr = myPort.readString();
      ParseInput(inStr);
    }
  }
  // check here timeout-ed end of binary data reception
  if (bin_rec && millis()-tim>TIMEOUT ) {
    bin_rec = false;
    // stop here the binary recording
    if ( data!=null ) saveBytes("RawWrite.txt", data);
    DisplayAddLine("binary recording ended, size of 'RawWrite.txt' is "+index,-1,0);
  }
}
/********************************************************************/
void ListRecordingOptions()
{
  String[] opt = {"Select option:","--------------------------------------",
                    "[0] : setup recording parameters","[1] : start recording","[2] : read recorded data"};
  DisplayAddLines(opt,2,1);
  DisplayClearLines();
  PromptReset();
  rec_ok = 0;
//  ShowScreen();
}
/********************************************************************/
String prompt = "> _"; int prPos = 2;
enum RecTarget { REC_TIME, SAMPLING_FREQ, CHANNELS_PER_SEQ};
RecTarget prTarget;
boolean showPrompt = false;
/********************************************************************/
void ShowPrompt()
{
  if ( !showPrompt ) return;
  stroke(0xff);  noFill();
  int y = 20*(disp_line)+5;
  println("drawing rectangle... disp_line: "+disp_line+", y: "+y);
  rect(10, y, 580, 20);
}
/********************************************************************/
void PromptReset() { prompt = "> _"; prPos = 2; }
/********************************************************************/
void PromptUpdate()
{
  if (whichKey=='\b') {
    // go back one character position and delete the last character form prompter line
    if ( (prPos)<3 ) return;	// do nothing
    prompt = prompt.substring(0,--prPos)+'_';
  } else if (whichKey>='0' && whichKey<='9') {
    prompt = prompt.substring(0,prPos++)+(char)whichKey+'_';
  } else if (whichKey=='\r' || whichKey=='\n') {
    // parse the input number
    String[] m = match(prompt, "> (\\d*)");
    if ( m!=null ) {
      int res = parseInt(m[1]); if ( res<=0 ) return;
      println("result of user input: "+res);
      switch (prTarget) {
        case REC_TIME:
          rec_time = res;
          String[] scr1 = {"- recording time: "+rec_time+" [ms]","","Enter sampling frequency [kHz]:"};
          DisplayAddLines(scr1,-1,-3);
          PromptReset();
          DisplayAddLine(prompt,-1,0);
          prTarget = RecTarget.SAMPLING_FREQ;
          rec_ok --;
          break;
        case SAMPLING_FREQ:
          sampling_freq = res;
          String[] scr2 = {"- sampling frequency: "+sampling_freq+" [kHz]","","Enter number of channels per sequence:"};
          DisplayAddLines(scr2,-1,-3);
          PromptReset();
          DisplayAddLine(prompt,-1,0);
          prTarget = RecTarget.CHANNELS_PER_SEQ;
          rec_ok --;
         break;
        case CHANNELS_PER_SEQ:  channels_per_seq = res;
          String[] scr3 = {"- channels per sequnece: "+channels_per_seq,"","Use these parameters to setup recording now? [y / n ]"};
          DisplayAddLines(scr3,-1,-3);
          showPrompt = false;
          break;
      }
      rec_ok ++;  // advance to next step in recording menu
    }
  }
  if (showPrompt) DisplayAddLine(prompt,-1,-1);
  else DisplayClearLines();
  println("PromptUpdate - prompt: "+prompt);  // debug
}
/********************************************************************/
void SetupRecording()
{
  //println("SetupRecording - rec_ok: "+rec_ok+", key: "+(char)whichKey);
  switch (rec_ok) {
    case 0:  // show recording parameter setop option
      if (whichKey<='0' && whichKey>='2') return;
      switch (whichKey) {
        case '0':  // recording parameter setup
          String[] scr = {"[0] - setup recording now using following (default) parameters:",
                          "          - recording time = 1000 ms",
                          "          - sampling frequency = 26 kHz","          - channels per sequence = 4",
                          "[1] - change parameters"};
          DisplayAddLines(scr,-1,-3);
          rec_ok ++;  // goto next parameter selection
          break;
        case '1':  // start recordings
          break;
        case '2':  // dump recorded data
          break;
      }
      break;
    case 1:  // display prompter rectangle
      switch (whichKey) {
        case '0':  // send the recording parameters to target
          break;
        case '1':  // show options menu
          String[] scr = {"Setup recording parameters","----------------------------------","","Enter recording time [ms]:"};
          DisplayAddLines(scr,2,-5);
          prTarget = RecTarget.REC_TIME;
          DisplayAddLine(prompt,-1,0);
          DisplayClearLines();
          showPrompt = true;
          rec_ok ++;  // goto next parameter selection
          break;
      } break;
    case 2:  // display prompter rectangle
      PromptUpdate();
      break;
    case 3:  // display prompter rectangle
      switch (whichKey) {
        case 'y':  // send the recording parameters to target
          break;
        case 'n':  // show options menu
          ListRecordingOptions();
          break;
      } break;
    case 14:
      break;
  }
}
/********************************************************************/
void ShowScreen()
{
  background(0);  // clear screen
  for (byte i=0; i<disp.length; i++) {
    if ( (disp[i]).length()>0)  text(disp[i], 10, 40+20*i);
  }
  //println("ShowScreen: disp_line = "+disp_line);
}
/********************************************************************/
void draw()
{
  if ( myKey>0 ) { // proces here the new pressed key
    whichKey = myKey;
    myKey = 0;
    if (whichKey<' ') println("*** pressed key: "+whichKey);  // debug
    else println("*** pressed key: "+(char)whichKey);  // debug
    if ( rec_ok>=0 )  SetupRecording();
    if ( serial_ok>=0 )  SetupSerial();
    ShowScreen();  // display text lines
    ShowPrompt();
    whichKey = 0;  // reset received key
  }
  CheckSerial();
}
/*********************************************************************/
void keyPressed() {
  // Send the keystroke out:
  myKey = key;
}