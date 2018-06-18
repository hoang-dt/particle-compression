%~dp0bgfx.cmake/build/install/bin/shaderc.exe -f vs_instancing.sc -o shaders/dx11/vs_instancing.bin -i %~dp0bgfx.cmake/bgfx/examples/common -i %~dp0bgfx.cmake\bgfx\src --type v --platform windows -p vs_4_0 --varyingdef varying.def.sc

%~dp0bgfx.cmake/build/install/bin/shaderc.exe -f fs_instancing.sc -o shaders/dx11/fs_instancing.bin -i %~dp0bgfx.cmake/bgfx/examples/common -i %~dp0bgfx.cmake\bgfx\src --type f --platform windows -p ps_4_0 --varyingdef varying.def.sc
