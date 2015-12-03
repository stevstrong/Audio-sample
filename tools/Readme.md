In order to get recorded data from the recording board to PC via the serial interface, an additional plugin is necessary:
https://github.com/billhsu/jUART.

The needed DLL: `/bin/Windows/npjUART.dll`

For installation, either
- copy `/bin/Windows/npjUART.dll` to `C:\Program Files (x86)\Mozilla Firefox\plugins`
or
- register the plugin DLL with regsvr32.exe: `regsvr32 npPluginTemplate.dll`
