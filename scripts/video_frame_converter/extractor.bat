:nextArgument
set arg=%~1
if not defined arg goto :eof2
call :processFile "%arg%"
shift
goto nextArgument

:processFile
set path=%~dpn1
if not exist "%path%" mkdir "%path%"
cd "%path%"

%~dp0\ffmpeg -i "%~1" -r 60/1 %%04d.jpg
goto :eof

:eof2
