@echo off
setlocal

pushd "%~dp0"

REM Pfad zur vendored premake-Binary, relativ zu diesem Skript (scripts\).
set "PREMAKE=..\vendor\bin\premake5.exe"

if not exist "%PREMAKE%" (
    echo [Setup] ERROR: %PREMAKE% was not found.
    echo [Setup] Please ensure premake5.exe is committed under vendor\bin\.
    popd
    exit /b 1
)

set ACTION=%1
if "%ACTION%"=="" set ACTION=vs2022

echo [Setup] Generating project files with action: %ACTION%
"%PREMAKE%" --file=..\premake5.lua %ACTION%
set EXITCODE=%ERRORLEVEL%

if %EXITCODE% neq 0 (
    echo [Setup] Premake failed with exit code %EXITCODE%.
    popd
    exit /b %EXITCODE%
)

echo [Setup] Done. Open Index-Physics.sln in Visual Studio.
popd
endlocal
