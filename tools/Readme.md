In order to get recorded data from the recording board to PC via the serial interface, an additional plugin is necessary:
https://github.com/billhsu/jUART.

The needed DLL: `/bin/Windows/npjUART.dll`

Direct link (for Windows): https://github.com/billhsu/jUART/blob/master/bin/Windows/npjUART.dll?raw=true

For installation, either
- copy the downloaded `npjUART.dll` to `C:\Program Files (x86)\Mozilla Firefox\plugins`

or (working for Windows 10)
- register the plugin DLL with regsvr32.exe: `regsvr32 npjUART.dll` (add the full path to the downloaded DLL file).
