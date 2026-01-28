@echo off
setlocal ENABLEDELAYEDEXPANSION

set "script_dir=%~dp0"
set "clean=false"

:: parse args
if not "%1" == "" (
    if "%1" == "-c" (
        set "clean=true"
    ) else (
        echo "Usage: %0 [-c]"
        exit /b 1
    )
)

:: clean if requested and setup
if exist "%script_dir%\build" (
    if "%clean%" == "true" (
        rmdir /s /q "%script_dir%\build"
        mkdir "%script_dir%\build"
    )
) else (
    mkdir "%script_dir%\build"
)

:: check for existing cmake system
cd /d "%script_dir%\build"
if exist "CMakeCache.txt" (
    goto build
)

:: find cmake generator
where ninja >nul 2>&1
if %ERRORLEVEL%==0 (
    set "generator=Ninja"
    goto generate
)
where make >nul 2>&1
if %ERRORLEVEL%==0 (
    set "generator=Unix Makefiles"
    goto generate
)
where mingw32-make >nul 2>&1
if %ERRORLEVEL%==0 (
    set "generator=MinGW Makefiles"
    goto generate
)
echo error: No cmake generator found.
exit /b 1

:generate
echo "generating cmake..."
echo "generator: %generator%"
cmake -G "%generator%" ..
if !errorlevel! neq 0 (
    cd ..
    exit /b 1
)

:build
echo; && echo "building..."
cmake --build .
cd ..
