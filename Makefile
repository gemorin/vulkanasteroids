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

MAX_SPRITES_PER_DRAW=10
SPRITE_TEXTURE_ARRAY_SIZE=7
MACROS=-DMAX_SPRITES_PER_DRAW=$(MAX_SPRITES_PER_DRAW) -DSPRITE_TEXTURE_ARRAY_SIZE=$(SPRITE_TEXTURE_ARRAY_SIZE)

INCS=-I$(GLFWDIR)/include/GLFW -I$(VULKANINCPATH)
CXXFLAGS=-Wall -W -g $(INCS) -std=c++14 -O2 -fno-exceptions $(MACROS)
LDFLAGS=-g -L$(GLFWDIR)/src -L $(VULKANLIBPATH) -framework Cocoa -framework Metal -framework IOSurface -rpath $(VULKANLIBPATH) -lglfw -lvulkan

all: vulkanasteroids fragment_background.spv vertex_background.spv vertex_sprite.spv fragment_sprite.spv vertex_explosions.spv fragment_explosions.spv vertex_overlay.spv fragment_overlay.spv vertex_hud.spv fragment_hud.spv
vulkanasteroids: vulkanasteroids.o mymath.o stb_image.o

stb_image.o: stb_image.h stb_image.cpp
vulkanasteroids.o: mymath.h stb_image.h
mymath.o: mymath.h mymath.cpp

fragment_background.spv: fragment_background.glsl
	$(SHADERCOMPILER) -fshader-stage=fragment -o $@ $<

vertex_background.spv: vertex_background.glsl
	$(SHADERCOMPILER) -fshader-stage=vertex -o $@ $<

fragment_sprite.spv: fragment_sprite.glsl Makefile
	$(SHADERCOMPILER) $(MACROS) -fshader-stage=fragment -o $@ $<

vertex_sprite.spv: vertex_sprite.glsl Makefile
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
