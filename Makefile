CXX=clang++
CC=clang++

GLFWDIR=/Users/guillaume/dev/glfw

#MOLTENDIR=/Users/guillaume/dev/MoltenVK
#MOLTENPKG=$(MOLTENDIR)/Package/Release
VULKANDIR=/Users/guillaume/dev/vulkansdk-macos-1.0.69.0
SHADERCOMPILER=$(VULKANDIR)/macOS/bin/glslc
MOLTENVK=$(VULKANDIR)/MoltenVK
VULKANLIBPATH=$(VULKANDIR)/macOS/lib

INCS=-I$(GLFWDIR)/include/GLFW -I$(MOLTENVK)/include -I../cubesolver/
CXXFLAGS=-Wall -W -g $(INCS) -std=c++14 -O2 -fno-exceptions
LDFLAGS=-L$(GLFWDIR)/src -L $(VULKANLIBPATH) -framework Cocoa -framework Metal -framework IOSurface -rpath $(VULKANLIBPATH) -lglfw -lvulkan

all: vulkanasteroids fragment.spv vertex.spv
vulkanasteroids: vulkanasteroids.o cube.o

cube.o: ../cubesolver/cube.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

fragment.spv: fragment.glsl
	$(SHADERCOMPILER) -fshader-stage=fragment -o $@ $<

vertex.spv: vertex.glsl
	$(SHADERCOMPILER) -fshader-stage=vertex -o $@ $<


clean:
	rm -rf *.o vulkanasteroids vulkanasteroids.dSYM *.spv
