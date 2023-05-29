cmake -S . -B build64 -A x64
cmake -S . -B build32 -A Win32

cd ..\..\

cd > _.tmp
set /p cur_dir= < _.tmp
del _.tmp
SET cur_dir2=%cur_dir:\=/%

cd c\windows

cd build32
echo srchrep.EXE -src "%cur_dir%" -dest "$(SolutionDir)\..\..\..\" *.vcxproj.*
echo srchrep.EXE -src "%cur_dir2%" -dest "$(SolutionDir)\..\..\..\" *.vcxproj.*
cd ..

cd build64
echo srchrep.EXE -src "%cur_dir%" -dest "$(SolutionDir)\..\..\..\" *.vcxproj.*
echo srchrep.EXE -src "%cur_dir2%" -dest "$(SolutionDir)\..\..\..\" *.vcxproj.*
cd ..
