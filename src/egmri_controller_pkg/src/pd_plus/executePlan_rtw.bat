@echo off

@if not "%MINGW_ROOT%" == "" (@set "PATH=%PATH%;%MINGW_ROOT%")

cd .

if "%1"=="" ("C:\Apps\R2020A~1\bin\win64\gmake"  -f executePlan_rtw.mk all) else ("C:\Apps\R2020A~1\bin\win64\gmake"  -f executePlan_rtw.mk %1)
@if errorlevel 1 goto error_exit

exit 0

:error_exit
echo The make command returned an error of %errorlevel%
exit 1
