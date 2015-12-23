/**
 * Serial Duplex 
 * by Tom Igoe. 
 * 
 * Sends a byte out the serial port when you type a key
 * listens for bytes received, and displays their value. 
 * This is just a quick application for testing serial data
 * in both directions. 
 */
import processing.serial.*;

Serial myPort;      // The serial port
String portName = "";
int bRate = 0; int dBits = 0; int par = 0; float sBits = 0;
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
PrintWriter output;
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
        DisplayAddLine("Selected serial port: "+portName,0,0);
        String[] scr1 = {"Select Baudrate:","[0] : 115k200","[1] : 250k000","[2] : 500k000"};
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
          DisplayAddLine("Selected Baudrate: "+bRate,1,0);
          String[] scr2 = {"Select data bits:","[0] : 7","[1] : 8"};
          DisplayAddLines(scr2,-1,0);
          serial_ok ++;  // goto next parameter selection
        }
        break;
    case 2:  // show selected data bits and parity options
        switch (whichKey) {
          case '0':  dBits = 7;  break;
          case '1':  dBits = 8;  break;
          default: break;
        }
        if ( dBits>0 ) {
          DisplayAddLine("Selected data bits: "+dBits,2,0);
          String[] scr3 = {"Select parity:","[0] : none","[1] : odd","[2] : even"};
          DisplayAddLines(scr3,-1,0);
          serial_ok ++;  // goto next parameter selection
        }
        break;
    case 3:  // show selected parity and stop bit options
        String p = "";
        switch (whichKey) {
          case '0':  par = 'N'; p = "none";  break;
          case '1':  par = 'O'; p = "odd"; break;
          case '2':  par = 'E'; p = "even"; break;
          default: break;
        }
        if ( par>0 ) {
          DisplayAddLine("Selected parity: "+p,3,0);
          String[] scr4 = {"Select stop bits:","[1] : 1","[2] : 2","[3] : 1.5"};
          DisplayAddLines(scr4,-1,0);
          serial_ok ++;  // goto next parameter selection
        }
        break;
    case 4:  // show selected data bits and parity options
        switch (whichKey) {
          case '1':  sBits = 1;  break;
          case '2':  sBits = 2;  break;
          case '3':  sBits = 1.5;  break;
          default: break;
        }
        if ( dBits>0 ) {
          DisplayAddLine("Selected stop bits: "+sBits,4,0);
          DisplayAddLine("Is the selection '"+portName+", "+bRate+", "+dBits+", "+(char)par+", "+sBits+"' correct? [ y / n ]",-1,0);
          DisplayClearLines();
          serial_ok ++;  // goto next parameter selection
        }
        break;
    case 5:  // show selected data bits and parity options
        switch (whichKey) {
          case 'y':  case 'Y':
            myPort = new Serial(this, portName, bRate, (char)par, dBits, sBits);  // Maple Mini analog recorder
            //myPort = new Serial(this, portName, 115200, 'E', 8, 2);  // EnergyCam
            if ( myPort!=null ) {
              DisplayAddLine("Serial port '"+portName+", "+bRate+", "+dBits+", "+(char)par+", "+sBits+"' opened successfully.",-1,-1);
              serial_ok ++;
              ListRecordingOptions();  // goto next parameter selection
            } else {
              DisplayAddLine("Opening port "+portName+", "+bRate+", "+dBits+", "+(char)par+", "+sBits+" ... failed!",-1,-1);
              DisplayAddLine("Check serial port and try again (press 'n')",-1,0);
            }
            break;
          case 'n':  case 'N':
              ListSerial(); break;
          default: break;
        }
        serial_ok ++;  // goto next parameter selection
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
  //println("_ "+str);
  //if (offset>0) println("offset="+offset);
  //if (line>=0)  println("line="+line);
  //println("adding display line at "+line+" with offset "+offset);
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
    //if ( disp_line<line )
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
    String[] rec00 = {"Select recording option:","[0] : setup recording parameters","[1] : start recording","[2] : read recorded data"};
    DisplayAddLines(rec00,-1,1);
	 rec_ok++;
//  ShowScreen();
}
/********************************************************************/
String prompt = "> _"; int prPos = 2;
enum rec_param_t { REC_TIME, SAMPLING_FREQ, CHANNELS_PER_SEQ};
rec_param_t prTarget;
/********************************************************************/
void PromptReset()
{
  prompt = "> _"; prPos = 1;
}
/********************************************************************/
void PromptUpdate()
{
  if (whichKey=='\b') {
    // go back one character position and delete the last character form prompter line
    if ( (prPos)<2 ) return;	// do nothing
    prompt = prompt.substring(0,--prPos)+'_';
	} else if (whichKey>='0' && whichKey<='9') {
      prompt = prompt.substring(0,prPos++)+(char)whichKey+'_';
	} else if (whichKey=='\r' || whichKey=='\n') {
    // parse the input number
    String[] m = match(promt, "> (\\d*)");
    if ( m!=null ) {
      int res = parseInt(m[1]); if ( r<=0 ) return;
      switch (prTarget) {
        case REC_TIME:  rec_time = res;	break;
        case SAMPLING_FREQ:  sampling_freq = res;	break;
        case CHANNELS_PER_SEQ:  channels_per_seq = res;	break;
      }				
      // advance to next step in recording menu
      rec_ok ++;
    }
	}
}
/********************************************************************/
void SetupRecording()
{
  switch (rec_ok) {
    case 0:  // show recording parameter setop option
        switch (whichKey) {
          case '0':
            DisplayAddLine("Enter recording time [ms]:",-1,1);
				prTarget = REC_TIME;
            break;
          case '1':
            DisplayAddLine("Enter sampling frequency [kHz]:",-1,1);
				prTarget = SAMPLING_FREQ;
            break;
          case '2':
            DisplayAddLine("Enter the number of channels per sequence:",-1,1);
				prTarget = CHANNELS_PER_SEQ;
            break;
          default: break;
        }
        rec_ok ++;  // goto next parameter selection
      break;
    case 1:  // show selected serial port and Baudrate options
	    // display prompter rectangle
	    stroke(0xff);
		 noFill();
		 rect(10, (disp_line-1)*20-10, 100, (disp_line-1)*20+10);
      break;
    case 2:
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
}
/********************************************************************/
void draw()
{
  if ( myKey>0 ) { // proces here the new pressed key
    whichKey = myKey;
    myKey = 0;
    println("*** pressed key: "+(char)whichKey);
    SetupSerial();
    ShowScreen();  // display text lines
    //text("serial_ok: "+serial_ok, 10, 360);  // debug
    whichKey = 0;  // reset received key
  }
  CheckSerial();
}
/*********************************************************************/
void keyPressed() {
  // Send the keystroke out:
  myKey = key;
}