@echo off
REM Starts a local web server (needed for microphone/audio access) and opens the app.
cd /d "%~dp0"
start "" http://localhost:8080/index.html
python -m http.server 8080
