@echo off
setlocal
cd /d "%~dp0"
x86_64-w64-mingw32-gcc -shared -O2 -DLINESIM_EXPORTS -o linesim.dll linesim.c -Wl,--out-implib,linesim.lib
if errorlevel 1 (
  echo Build failed. Install MinGW-w64 or rebuild linesim.c with your C compiler/IDE.
  exit /b 1
)
