@echo off
setlocal enabledelayedexpansion

set ROOT_DIR=%~dp0
set PYTHON_BIN=

if exist "%ROOT_DIR%\.venv\Scripts\python.exe" (
  set PYTHON_BIN=%ROOT_DIR%\.venv\Scripts\python.exe
) else (
  where python >nul 2>nul
  if %errorlevel%==0 (
    set PYTHON_BIN=python
  )
)

if "%PYTHON_BIN%"=="" (
  echo Python not found. Install Python or create .venv in %ROOT_DIR%.
  exit /b 1
)

set PYTHON=%PYTHON_BIN%

"%PYTHON_BIN%" -m pip install -r "%ROOT_DIR%requirements.txt"
call npm install

start "" /b "%PYTHON_BIN%" "%ROOT_DIR%backend_server.py"
set START_BACKEND=0
call npm start
