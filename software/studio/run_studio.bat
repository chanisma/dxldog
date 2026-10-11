@echo off
rem Launch DXL Dog Studio (editor + 3D robot view).
rem   run_studio.bat                              simulator
rem   run_studio.bat --backend real --port COM3   real robot (U2D2)
rem   run_studio.bat --backend real --port fake   fake motors (practice without hardware)
rem   run_studio.bat --robot sts ...              robot type: mixed (DYNAMIXEL, default) or sts (Feetech STS3215)
start "" "%~dp0..\..\.venv\Scripts\pythonw.exe" "%~dp0studio_web.py" %*
