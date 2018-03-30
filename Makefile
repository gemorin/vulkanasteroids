CXX=clang++
CC=clang++

GLFWDIR=/Users/guillaume/dev/glfw


VULKANDIR=/Users/guillaume/dev/vulkansdk-macos-1.0.69.0
SHADERCOMPILER=$(VULKANDIR)/macOS/bin/glslc
VULKANINCPATH=$(VULKANDIR)/MoltenVK/include
VULKANLIBPATH=$(VULKANDIR)/macOS/lib

#VULKANDIR=/Users/guillaume/dev/MoltenVK/Package/Debug/MoltenVK
VULKANDIR=/Users/guillaume/dev/Vulkan-LoaderAndValidationLayers
VULKANINCPATH=$(VULKANDIR)/external/MoltenVK/Package/Release/MoltenVK/include/
VULKANLIBPATH=$(VULKANDIR)/build/loader
SHADERCOMPILER=/Users/guillaume/dev/vulkansdk-macos-1.0.69.0/macOS/bin/glslc

NUM_MAX_ASTEROIDS=10
TEXTURE_ARRAY_SIZE=7
MACROS=-DNUM_MAX_ASTEROIDS=$(NUM_MAX_ASTEROIDS) -DTEXTURE_ARRAY_SIZE=$(TEXTURE_ARRAY_SIZE)

INCS=-I$(GLFWDIR)/include/GLFW -I$(VULKANINCPATH)
CXXFLAGS=-Wall -W -g $(INCS) -std=c++14 -O2 -fno-exceptions $(MACROS)
LDFLAGS=-g -L$(GLFWDIR)/src -L $(VULKANLIBPATH) -framework Cocoa -framework Metal -framework IOSurface -rpath $(VULKANLIBPATH) -lglfw -lvulkan

all: vulkanasteroids fragment_background.spv vertex_background.spv vertex_ship.spv fragment_ship.spv vertex_explosions.spv fragment_explosions.spv vertex_overlay.spv fragment_overlay.spv vertex_hud.spv fragment_hud.spv
vulkanasteroids: vulkanasteroids.o mymath.o stb_image.o

stb_image.o: stb_image.h stb_image.cpp
vulkanasteroids.o: mymath.h stb_image.h
mymath.o: mymath.h mymath.cpp

fragment_background.spv: fragment_background.glsl
	$(SHADERCOMPILER) -fshader-stage=fragment -o $@ $<

vertex_background.spv: vertex_background.glsl
	$(SHADERCOMPILER) -fshader-stage=vertex -o $@ $<

fragment_ship.spv: fragment_ship.glsl Makefile
	$(SHADERCOMPILER) $(MACROS) -fshader-stage=fragment -o $@ $<

vertex_ship.spv: vertex_ship.glsl Makefile
	$(SHADERCOMPILER) $(MACROS) -fshader-stage=vertex -o $@ $<

fragment_explosions.spv: fragment_explosions.glsl
	$(SHADERCOMPILER) -fshader-stage=fragment -o $@ $<

vertex_explosions.spv: vertex_explosions.glsl
	$(SHADERCOMPILER) -fshader-stage=vertex -o $@ $<

fragment_overlay.spv: fragment_overlay.glsl
	$(SHADERCOMPILER) -fshader-stage=fragment -o $@ $<

vertex_overlay.spv: vertex_overlay.glsl
	$(SHADERCOMPILER) -fshader-stage=vertex -o $@ $<

fragment_hud.spv: fragment_hud.glsl
	$(SHADERCOMPILER) -fshader-stage=fragment -o $@ $<

vertex_hud.spv: vertex_hud.glsl
	$(SHADERCOMPILER) -fshader-stage=vertex -o $@ $<

clean:
	rm -rf *.o vulkanasteroids vulkanasteroids.dSYM *.spv
