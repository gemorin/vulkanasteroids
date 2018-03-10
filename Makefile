CXX=clang++
CC=clang++

GLFWDIR=/Users/guillaume/dev/glfw

MOLTENDIR=/Users/guillaume/dev/MoltenVK
MOLTENPKG=$(MOLTENDIR)/Package/Release
SHADERCOMPILER=$(MOLTENPKG)/MoltenVKShaderConverter/Tools/MoltenVKShaderConverter
MOLTENVK=$(MOLTENPKG)/MoltenVK

INCS=-I$(GLFWDIR)/include/GLFW -I$(MOLTENVK)/include -I../cubesolver/
CXXFLAGS=-Wall -W -g $(INCS) -std=c++14 -O2 -fno-exceptions
LDFLAGS=-L$(GLFWDIR)/src -L$(MOLTENVK)/macOS -framework Cocoa -framework Metal -framework IOSurface -rpath $(MOLTENVK)/macOS -lMoltenVK -lglfw

all: vulkanasteroids fragment.spv vertex.spv
vulkanasteroids: vulkanasteroids.o cube.o

cube.o: ../cubesolver/cube.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

fragment.spv: fragment.glsl
	$(SHADERCOMPILER) -t f -gi $< -so $@

vertex.spv: vertex.glsl
	$(SHADERCOMPILER) -t v -gi $< -so $@


clean:
	rm -rf *.o vulkanasteroids vulkanasteroids.dSYM *.spv
