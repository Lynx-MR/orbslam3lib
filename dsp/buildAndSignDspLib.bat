call .\setupEnv.bat
build_cmake hexagon DSP_ARCH=v66 BUILD=Release
python.exe %HEXAGON_SDK_ROOT%\utils\scripts\signer.py -i .\hexagon_Release_toolv87_v66\ship\liborbslam3_skel.so
copy /Y .\output\liborbslam3_skel.so ..\..\jniLibs\arm64-v8a\
exit 0
