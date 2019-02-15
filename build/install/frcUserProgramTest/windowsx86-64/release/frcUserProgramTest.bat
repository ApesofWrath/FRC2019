
@echo off
SETLOCAL

CALL "%~dp0lib\frcUserProgramTest.exe" %*
EXIT /B %ERRORLEVEL%
ENDLOCAL
