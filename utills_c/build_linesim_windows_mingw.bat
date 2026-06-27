@echo off
setlocal
cd /d "%~dp0"
where gcc >nul 2>nul
if errorlevel 1 (
  echo Build failed. Open MSYS2 UCRT64 or install MinGW-w64 GCC.
  exit /b 1
)
gcc -shared -O2 -o linesim.dll linesim.c -Wl,--out-implib,linesim.lib -static-libgcc
if errorlevel 1 (
  echo Build failed. Check compiler output above.
  exit /b 1
)
echo Built %cd%\linesim.dll
endlocal
