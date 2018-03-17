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

INCS=-I$(GLFWDIR)/include/GLFW -I$(VULKANINCPATH) -I../cubesolver/
CXXFLAGS=-Wall -W -g $(INCS) -std=c++14 -O2 -fno-exceptions
LDFLAGS=-g -L$(GLFWDIR)/src -L $(VULKANLIBPATH) -framework Cocoa -framework Metal -framework IOSurface -rpath $(VULKANLIBPATH) -lglfw -lvulkan

all: vulkanasteroids fragment_background.spv vertex_background.spv vertex_ship.spv fragment_ship.spv
vulkanasteroids: vulkanasteroids.o cube.o

cube.o: ../cubesolver/cube.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

fragment_background.spv: fragment_background.glsl
	$(SHADERCOMPILER) -fshader-stage=fragment -o $@ $<

vertex_background.spv: vertex_background.glsl
	$(SHADERCOMPILER) -fshader-stage=vertex -o $@ $<

fragment_ship.spv: fragment_ship.glsl
	$(SHADERCOMPILER) -fshader-stage=fragment -o $@ $<

vertex_ship.spv: vertex_ship.glsl
	$(SHADERCOMPILER) -fshader-stage=vertex -o $@ $<

clean:
	rm -rf *.o vulkanasteroids vulkanasteroids.dSYM *.spv
