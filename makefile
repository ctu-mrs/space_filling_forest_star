TARGET := main
ZIP := files.zip
SOURCES := $(wildcard src/*.cpp)
SOURCES := $(SOURCES:src/%=%)
FILES := $(wildcard src/*.cpp) $(wildcard src/*.h)
OBJECTS := $(patsubst %.c,%.o, $(patsubst %.cpp,%.o,$(SOURCES)))
DEP := test.xml
OBJDIR := obj
DBGDIR := debug
RELDIR := release

INCLUDE := -I. -I./lib/rapid-2.01 -I./lib/ann/ann/include -I./lib/rapidxml
LIBPATH := -L./lib/ -L./lib/ann/ -L./lib/flann/ -L./lib/rapid-2.01 -L./lib/rapidxml
LIBS := -lgmp -lRAPID -llz4 

CXXFLAGS := -std=c++17
CXX := g++

DBGEXE := $(DBGDIR)/$(TARGET)
DBGOBJS := $(addprefix $(OBJDIR)/, $(addprefix $(DBGDIR)/, $(OBJECTS)))
#DBGFLAGS := $(CXXFLAGS) -g -fsanitize=address -O0 -fno-omit-frame-pointer
DBGFLAGS := $(CXXFLAGS) -g -O0

RELEXE := $(RELDIR)/$(TARGET)
RELOBJS := $(addprefix $(OBJDIR)/, $(addprefix $(RELDIR)/, $(OBJECTS)))
RELFLAGS := $(CXXFLAGS) -O3

.PHONY: all clean debug release prep rapid flann zip

all: release zip

############ DEBUG ###############
debug: prep $(DBGEXE)

$(DBGEXE): $(DBGOBJS)
	$(CXX) $(DBGFLAGS) $(INCLUDE) -o $(DBGEXE) $^ $(LIBPATH) $(LIBS)
ifneq ("$(wildcard test.xml)", "")
	@cp $(DEP) $(DBGDIR)/$(DEP)
endif

$(OBJDIR)/$(DBGDIR)/%.o: src/%.cpp
	$(CXX) $(DBGFLAGS) $(INCLUDE) -c $< -o $@

############ RELEASE ###############
release: prep $(RELEXE)

$(RELEXE): $(RELOBJS)
	$(CXX) $(RELFLAGS) $(INCLUDE) -o $(RELEXE) $^ $(LIBPATH) $(LIBS)
ifneq ("$(wildcard test.xml)", "")
	@cp $(DEP) $(RELDIR)/$(DEP)
endif

$(OBJDIR)/$(RELDIR)/%.o: src/%.cpp
	$(CXX) $(RELFLAGS) $(INCLUDE) -c $< -o $@

############# COMMON ##############	
prep:
	@echo "Creating all dirs.."
	@mkdir -p $(OBJDIR)
	@mkdir -p $(OBJDIR)/$(DBGDIR)
	@mkdir -p $(OBJDIR)/$(RELDIR)
	@mkdir -p $(DBGDIR)
	@mkdir -p $(RELDIR)

rapid:
	@echo "Installing RAPID..."
	@$(MAKE) -C lib/rapid-2.01/

flann:
	@echo "Installing FLANN..."
	@cd ./lib/flann; mkdir build; cd build; cmake .. -DCMAKE_INSTALL_PREFIX=../install; make -j4; make install

zip:
	@zip -u $(ZIP) $(FILES) makefile

clean:
	@echo "Cleaning..."
	@rm -rf $(OBJDIR)
	@rm -rf $(RELDIR)
	@rm -rf $(DBGDIR)
