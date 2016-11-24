/**
 * Serial Data Dumper 
 * 
 * Opens a serial connection and communicates with Maple Mini analog recorder.
 * Features:
 * - setup serial port parameters
 * - menu to choose one of the recording options:
 *   - setup recording parameters (recording time, sampling frequency, channels per sequence)
 *   - start recording
 *   - dump recorded data
 *
 */
import processing.serial.*;

int BUFF_SIZE = 2048;
Serial myPort;      // The serial port
String portName = "";
int bRate = 0;
int whichKey = 0; int myKey = 0; // Variable to hold keystoke value
int[] keys = new int[5];  // max 5 key possibilities
int inByte = -1;    // Incoming serial data
int serial_ok = -1;
int rec_ok = -1;
byte[] inBuffer = new byte[BUFF_SIZE];
int index = 0;
int cnt = 0;
int tim = 0;
int TIMEOUT = 100;  // millis
int rec_time = 1000, sampling_freq = 26, samples_per_seq = 4;
boolean bin_rec;
int bin_len;
byte[] data = new byte[520];
byte screen = 0;
String[] disp = new String[30];
int status = 0;
/*********************************************************************/
void setup()
{
  size(800, 700);
  background(0);
  // create a font with the third font available to the system:
  //PFont myFont = createFont(PFont.list()[8], 14);
  PFont myFont = createFont("Courier new", 16);
  textFont(myFont);
  for (int i=0;i<disp.length; i++)  disp[i] = "";  // initialize string array
  ListSerial();  // init serial port
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
}
/********************************************************************/
void CheckSerial()
{
	if ( serial_ok<2 ) return;  // do it only if serial setup was successful.

	if ( myPort.available()<=0 ) return;	// no data available

	if ( bin_rec ) {  // binary reception
		ParseBinaryData();
	} else {  // string reception
		//String inStr = myPort.readString();	ParseStringData(inStr);
		//ParseStringData(myPort.readString());
		ParseStringData(myPort.readBytesUntil('\n', inBuffer));
	}
}
/********************************************************************
arrayCopy(src, srcPosition, dst, dstPosition, length)
arrayCopy(src, dst, length)
********************************************************************/
/********************************************************************/
/* Binary packet format:
start_id	[0x01]
data_length	[len_high] [len_low]
payload		[dd] ... [dd]
crc			[crc_high] [crc_low]
/********************************************************************/
void Tick() {
	int tim1 = millis();
	print('.');
	while ( (millis()-tim1)<20 );	// wait some time
}
/********************************************************************/
/********************************************************************/
int GetByteFromSerial()
{
	int tim2 = millis();
	while ( myPort.available()<=0 ) {
		Tick();
		if ( (millis()-tim2)>1000 ) {
			println("! timeout receiving data from serial!");
			return -1;
		};
	}	// wait to receive something
	int c = myPort.read();
	//println("serial data: 0x"+hex(c,2));	// debug
	return c;
}
/********************************************************************/
//int len, pack_len;	// payload index
/********************************************************************/
int GetPayload(int bytes)
{
	int c;
	int pl_index = 0;
	while ( bytes>0 ) {
		c = GetByteFromSerial();
		if ( c<0 ) {
			println("! incomplete payload, received bytes: "+pl_index);
			return 1;
		}
		inBuffer[pl_index++] = (byte)c;
		bytes --;
	}
	//println("got payload bytes: "+pl_index);
	//print('.');
	return 0;
}
/********************************************************************/
/********************************************************************/
void ParseBinaryData()
{
	// read the available bytes
	int a, len;
	//while (true) {
		// get header ID
		if ( (a=GetByteFromSerial())<0 ) {
			print("Error receiving binary data!");
			bin_rec = false;
			return;
		}

		if ( a==0x01 ) {	// parse header info
			// get here payload length
			len = GetByteFromSerial();
			len += GetByteFromSerial()<<8;
			//println("payload length: "+len);	// debug
			// wait for entire payload reception
			if ( GetPayload(len)>0 ) {
				println("\n<-> payload reception error <->\n");
				bin_rec = false;
				return;
			}
	/**/
			if ( (index+len)<=data.length ) {
				arrayCopy(inBuffer,0,data,index,len);
				index += len;  // update data index
				DisplayAddLine("index = "+index,-1,-1);
			} else {
				println("!data buffer overflow!");
				DisplayAddLine("!data buffer overflow!",-1,0);
			}
		
		} else if ( a==0x17 ) {	// end of transmission block
			// stop here the binary recording
			tim = (millis() - tim);
			//println("time diff: "+tim);
			if ( data!=null ) saveBytes("RawWrite.txt", data);
			println("\nfinished receiving 'RawWrite.txt' of size "+index+" in "+tim+" millis.");
			DisplayAddLine("finished receiving 'RawWrite.txt' of size "+index+" in "+tim+" millis.",-1,0);
			bin_rec = false;
			bin_len = 0;
			//break;
		} else {
			println("!wrong binary ID received: 0x"+hex(a,2));
			//DisplayAddLine("!wrong binary ID!",-1,0); 
			return;
		}
	//}
/*
  if ( data==null || index==0) {
    data = new byte[len];
    arrayCopy(inBuffer, data);
  } else
    data = concat(data, inBuffer);  // add new bytes to the data array
  index = data.length;
*/
}
/********************************************************************/
String[] q,m;
/********************************************************************/
//void ParseStringData(String inStr)
void ParseStringData(int nrBytes)
{
	if ( nrBytes<2 ) return;
	//q = splitTokens(inStr, "\r\n");
	//for (byte i=0; i<q.length; i++) {
		//DisplayAddLine(i+": "+q[i], -1,0);  // display received serial data
		String inStr = new String(inBuffer, 0, nrBytes-1);
		println("> "+inStr);
		// check binary marker
		if ( inStr.indexOf(">>>")>=0 ) {
			println("binary transmission active");
			DisplayAddLine("receiving data...", -1,0);
			bin_rec = true;
			index = 0;
			// send the acknowledge byte
			//println("Sending ACK");
			//myPort.buffer(1);
			myPort.write(0x06);
			tim = millis();	// start measuring time

		} else if (bin_len==0) {
			// parse the binary length
			m = match(inStr, "binary_length:\\s*(\\d*)");
			if ( m!=null ) {
				bin_len = parseInt(m[1]);
				println("bin_len = "+bin_len);
				DisplayAddLine("bin_len = "+bin_len,-1,0);
				data = new byte[bin_len];
			}
		}
	//}
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
                            "-----------------------------------------------------------------",
                          "[0] : 115200, 8, N, 1","[1] : 250000, 8, N, 1","[2] : 500000, 8, N, 1"};
        DisplayAddLines(scr1,-1,0);
        serial_ok ++;  // goto next parameter selection
      }
      break;
    case 1:  // show selected Baud rate and data bits option
      switch (whichKey) {
        case '0':  bRate = 115200;  break;
        case '1':  bRate = 250000;  break;
        case '2':  bRate = 500000;  break;
        default:   bRate = 0; break;
      }
      if ( bRate>0 ) {
          myPort = new Serial(this, portName, bRate, 'N', 8, 1);  // Maple Mini analog recorder
          //myPort = new Serial(this, portName, 115200, 'E', 8, 2);  // EnergyCam
          if ( myPort!=null ) {
            DisplayAddLine("Serial port "+portName+", "+bRate+", 8, 'N', 1 opened successfully.",0,0);
            DisplayClearLines();
            ListRecordingOptions();
            serial_ok ++;  // goto next parameter selection
			myPort.bufferUntil('\n');
          } else {
            DisplayAddLine("Opening port "+portName+", "+bRate+", 8, 'N', 1 ... failed!",-1,-1);
            DisplayAddLine("Check serial port and try again (press 'n')",-1,0);
          }
      }
      break;
    default:  break;
  }
}
/********************************************************************/
void SendRecordingConfig() {
  myPort.write("set rec_time="+rec_time+";sampling_freq="+sampling_freq+";samples_per_seq="+samples_per_seq+"\n");
}
/********************************************************************/
void SendRecordingStart() {
  myPort.write("go\n");
}
/********************************************************************/
void SendRecordingGetData() {
  myPort.write("get\n");
}/********************************************************************/
int disp_line = 0;
/********************************************************************/
void DisplayAddLine(String str, int line, int offset)
{
  //println("display in line "+disp_line+": "+str);  // debug
  int dLen = disp.length;
  int scroll = (disp_line+1+offset)-dLen;
  if ( scroll>0) {
    disp_line = dLen-1;
    //println("scroll is: "+scroll);
  }
  while ( line==-1 && scroll>0 ) {  // have to scroll?
    for (int i=8; i<dLen-1; i++) disp[i] = disp[i+1];  // scroll all lines one up
    scroll--;
  }
  if ( line==-1 ) {  // add to last available line
    disp[disp_line+=offset] = str;
    if ((++disp_line)>dLen) disp_line = dLen;  // limit check for line
  } else {
    disp[line] = str;
    disp_line = line+1;
  }
  ShowScreen();  // refresh display
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
  PromptReset();
}
/********************************************************************/
void ListRecordingOptions()
{
  String[] opt = {"Select option:","--------------------------------------",
    "[0] : setup recording parameters","[1] : start recording","[2] : read recorded data",
    "--------------------------------------"};
  DisplayAddLines(opt,2,1);
  DisplayClearLines();
  rec_ok = 0;
//  ShowScreen();
}
/********************************************************************/
String prompt = "> _"; int prPos = 2;
enum RecTarget { REC_TIME, SAMPLING_FREQ, SAMPLES_PER_SEQ};
RecTarget prTarget;
boolean showPrompt = false;
/********************************************************************/
void ShowPrompt()
{
  if ( !showPrompt ) return;
  stroke(0xff);  noFill();
  int y = 20*(disp_line)+5;
  //println("drawing rectangle... disp_line: "+disp_line+", y: "+y);  // debug
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
          String[] scr2 = {"- sampling frequency: "+sampling_freq+" [Hz]","","Enter number of channels per sequence:"};
          DisplayAddLines(scr2,-1,-3);
          PromptReset();
          DisplayAddLine(prompt,-1,0);
          prTarget = RecTarget.SAMPLES_PER_SEQ;
          rec_ok --;
         break;
        case SAMPLES_PER_SEQ:
          samples_per_seq = res;
          String[] scr3 = {"- samples per sequence: "+samples_per_seq,"","Press '0' to return to previous menu."};
          DisplayAddLines(scr3,-1,-3);
          rec_ok = -1; //whichKey = '0';  // force menu
          showPrompt = false;
          break;
      }
      rec_ok ++;  // advance to next step in recording menu
    }
  }
  if (showPrompt) DisplayAddLine(prompt,-1,-1);
  else DisplayClearLines();
  //println("PromptUpdate - prompt: "+prompt);  // debug
}
/********************************************************************/
void SetupRecording()
{
  //println("SetupRecording - rec_ok: "+rec_ok+", key: "+(char)whichKey);
  switch (rec_ok) {
    case 0:  // show recording parameter setop option
      switch (whichKey) {
        case '0':  // recording parameter setup
          String[] scr = {"[0] - setup recording now using following parameters:",
                          "          - recording time = "+rec_time+" ms",
                          "          - sampling frequency = "+sampling_freq+" Hz",
                          "          - samples per sequence = "+samples_per_seq,
                          "[1] - change parameters","[2] : return","--------------------------------------"};
          DisplayAddLines(scr,4,0);
          DisplayClearLines();
          rec_ok ++;  // goto next parameter selection
          break;
        case '1':  // start recordings
          SendRecordingStart();
          break;
        case '2':  // dump recorded data
          SendRecordingGetData();
          break;
      }
      break;
    case 1:  // communicate with the target
      switch (whichKey) {
        case '0':  // send the recording parameters to target
          ListRecordingOptions();
          SendRecordingConfig();
          break;
        case '1':  // show options menu
          String[] scr = {"Setup recording parameters","--------------------------------------","","Enter recording time [ms]:"};
          DisplayAddLines(scr,2,-5);
          prTarget = RecTarget.REC_TIME;
          DisplayAddLine(prompt,-1,0);
          DisplayClearLines();
          showPrompt = true;
          rec_ok ++;  // goto next parameter selection
          break;
        case '2':  // return
          ListRecordingOptions();
          break;
      } break;
    case 2:  // prompt update
      PromptUpdate();
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
  if ( myKey>0 ) { // process here the new pressed key
    whichKey = myKey;
    myKey = 0;
    if (whichKey<' ') println("pressed key: "+whichKey);  // debug
    else println("pressed key: '"+(char)whichKey+"'");  // debug
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
