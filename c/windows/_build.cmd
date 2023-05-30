SET VS2019=C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional
SET MSBUILD="%VS2019%\MSBuild\Current\Bin\MSBuild.exe"
%MSBUILD%  build64\ui2c_project.sln -target:ui2c_shared  /property:Configuration="Release"
%MSBUILD%  build64\ui2c_project.sln -target:ui2c_static  /property:Configuration="Release"
%MSBUILD%  build64\ui2c_project.sln -target:test_app     /property:Configuration="Release"

%MSBUILD%  build32\ui2c_project.sln -target:ui2c_shared  /property:Configuration="Release"
%MSBUILD%  build32\ui2c_project.sln -target:ui2c_static  /property:Configuration="Release"
%MSBUILD%  build32\ui2c_project.sln -target:test_app     /property:Configuration="Release"

