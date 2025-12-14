# Makefile for Social Network Analysis Project
# Compiler settings
CXX = g++
CXXFLAGS = -std=c++11 -O2 -Wall -Wextra

# Target executable
TARGET = SocialNetworkAnalysis
SOURCE = SocialNetworkAnalysis.cpp

# Default target
all: $(TARGET)

# Compile the main program
$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE)

# Clean compiled files
clean:
	rm -f $(TARGET)

# Run the program
run: $(TARGET)
	./$(TARGET)

# Windows specific run
run-windows: $(TARGET).exe
	$(TARGET).exe

# Windows executable
$(TARGET).exe: $(SOURCE)
	$(CXX) $(CXXFLAGS) -o $(TARGET).exe $(SOURCE)

.PHONY: all clean run run-windows