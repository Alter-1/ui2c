SET VS2019=C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional
SET MSBUILD="%VS2019%\MSBuild\Current\Bin\MSBuild.exe"
%MSBUILD%  build64\ui2c_project.sln -target:ui2c_shared  /property:Configuration="Release"
%MSBUILD%  build64\ui2c_project.sln -target:ui2c_static  /property:Configuration="Release"
%MSBUILD%  build64\ui2c_project.sln -target:test_app     /property:Configuration="Release"

%MSBUILD%  build32\ui2c_project.sln -target:ui2c_shared  /property:Configuration="Release"
%MSBUILD%  build32\ui2c_project.sln -target:ui2c_static  /property:Configuration="Release"
%MSBUILD%  build32\ui2c_project.sln -target:test_app     /property:Configuration="Release"


SET DSTDIR=..\..\bin
if not exist %DSTDIR% mkdir %DSTDIR%

SET DSTDIR1=%DSTDIR%\Win32
if not exist %DSTDIR1% mkdir %DSTDIR1%
copy build32\bin\Release\* %DSTDIR1%
copy build32\lib\Release\*.lib %DSTDIR1%

SET DSTDIR1=%DSTDIR%\x64
if not exist %DSTDIR1% mkdir %DSTDIR1%
copy build64\bin\Release\* %DSTDIR1%
copy build64\lib\Release\*.lib %DSTDIR1%
