###########################################################################
## Makefile generated for MATLAB file/project 'executePlan'. 
## 
## Makefile     : executePlan_rtw.mk
## Generated on : Wed Dec 01 14:08:21 2021
## MATLAB Coder version: 5.0 (R2020a)
## 
## Build Info:
## 
## Final product: ./executePlan.lib
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPILER_COMMAND_FILE   Compiler command listing model reference header paths
# CMD_FILE                Command file
# MODELLIB                Static library target

PRODUCT_NAME              = executePlan
MAKEFILE                  = executePlan_rtw.mk
MATLAB_ROOT               = C:/Apps/R2020A~1
MATLAB_BIN                = C:/Apps/R2020A~1/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/win64
MASTER_ANCHOR_DIR         = 
START_DIR                 = C:/Users/segigar/ONEDRI~1/Desktop/ABB/test/GENCOD~1/codegen/lib/EXECUT~1
TGT_FCN_LIB               = ISO_C++
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = .
COMPILER_COMMAND_FILE     = executePlan_rtw_comp.rsp
CMD_FILE                  = executePlan_rtw.rsp
C_STANDARD_OPTS           = -fwrapv
CPP_STANDARD_OPTS         = -fwrapv
MODELLIB                  = executePlan.lib

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          MinGW64 | gmake (64-bit Windows)
# Supported Version(s):    6.x
# ToolchainInfo Version:   2020a
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# C_STANDARD_OPTS
# CPP_STANDARD_OPTS
# MINGW_ROOT
# MINGW_C_STANDARD_OPTS

#-----------
# MACROS
#-----------

WARN_FLAGS            = -Wall -W -Wwrite-strings -Winline -Wstrict-prototypes -Wnested-externs -Wpointer-arith -Wcast-align
WARN_FLAGS_MAX        = $(WARN_FLAGS) -Wcast-qual -Wshadow
CPP_WARN_FLAGS        = -Wall -W -Wwrite-strings -Winline -Wpointer-arith -Wcast-align
CPP_WARN_FLAGS_MAX    = $(CPP_WARN_FLAGS) -Wcast-qual -Wshadow
MW_EXTERNLIB_DIR      = $(MATLAB_ROOT)/extern/lib/win64/mingw64
SHELL                 = %SystemRoot%/system32/cmd.exe

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lws2_32

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: GNU C Compiler
CC_PATH = $(MINGW_ROOT)
CC = "$(CC_PATH)/gcc"

# Linker: GNU Linker
LD_PATH = $(MINGW_ROOT)
LD = "$(LD_PATH)/g++"

# C++ Compiler: GNU C++ Compiler
CPP_PATH = $(MINGW_ROOT)
CPP = "$(CPP_PATH)/g++"

# C++ Linker: GNU C++ Linker
CPP_LD_PATH = $(MINGW_ROOT)
CPP_LD = "$(CPP_LD_PATH)/g++"

# Archiver: GNU Archiver
AR_PATH = $(MINGW_ROOT)
AR = "$(AR_PATH)/ar"

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = %MATLAB%/bin/win64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @del
ECHO                = @echo
MV                  = @move
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = ruvs
CFLAGS               = -c $(MINGW_C_STANDARD_OPTS) -m64 \
                       -O3 -fno-loop-optimize -fno-aggressive-loop-optimizations
CPPFLAGS             = -c $(CPP_STANDARD_OPTS) -m64 -std=c++11 \
                       -O3 -fno-loop-optimize -fno-aggressive-loop-optimizations
CPP_LDFLAGS          = -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" -static -m64
CPP_SHAREDLIB_LDFLAGS  = -shared -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" -Wl,--no-undefined \
                         -Wl,--out-implib,$(basename $(PRODUCT)).lib
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" -static -m64
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" -Wl,--no-undefined \
                       -Wl,--out-implib,$(basename $(PRODUCT)).lib



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./executePlan.lib
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = 

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -D__USE_MINGW_ANSI_STDIO=1 -DMODEL=executePlan
DEFINES_CUSTOM = 
DEFINES_STANDARD = -DMODEL=executePlan

DEFINES = $(DEFINES_) $(DEFINES_CUSTOM) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/executePlan_data.cpp $(START_DIR)/executePlan_initialize.cpp $(START_DIR)/executePlan_terminate.cpp $(START_DIR)/executePlan.cpp $(START_DIR)/controller.cpp $(START_DIR)/deal.cpp $(START_DIR)/ForwardKinematics.cpp $(START_DIR)/eye.cpp $(START_DIR)/ListGet.cpp $(START_DIR)/Joint.cpp $(START_DIR)/any1.cpp $(START_DIR)/allOrAny.cpp $(START_DIR)/applyVectorFunction.cpp $(START_DIR)/all.cpp $(START_DIR)/Skew.cpp $(START_DIR)/mtimes.cpp $(START_DIR)/mtimes1.cpp $(START_DIR)/abs.cpp $(START_DIR)/abs1.cpp $(START_DIR)/sqrt.cpp $(START_DIR)/sqrt1.cpp $(START_DIR)/Quaternion.cpp $(START_DIR)/DerProd.cpp $(START_DIR)/Exponential.cpp $(START_DIR)/sin.cpp $(START_DIR)/sin1.cpp $(START_DIR)/cos.cpp $(START_DIR)/cos1.cpp $(START_DIR)/InvAdjoint.cpp $(START_DIR)/DerSum.cpp $(START_DIR)/ListSet.cpp $(START_DIR)/DynamicMatrices.cpp

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = executePlan_data.obj executePlan_initialize.obj executePlan_terminate.obj executePlan.obj controller.obj deal.obj ForwardKinematics.obj eye.obj ListGet.obj Joint.obj any1.obj allOrAny.obj applyVectorFunction.obj all.obj Skew.obj mtimes.obj mtimes1.obj abs.obj abs1.obj sqrt.obj sqrt1.obj Quaternion.obj DerProd.obj Exponential.obj sin.obj sin1.obj cos.obj cos1.obj InvAdjoint.obj DerSum.obj ListSet.obj DynamicMatrices.obj

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)

CPPFLAGS += $(CPPFLAGS_BASIC)

#---------------------
# MEX C++ Compiler
#---------------------

MEX_CPP_Compiler_BASIC =  @$(COMPILER_COMMAND_FILE)

MEX_CPPFLAGS += $(MEX_CPP_Compiler_BASIC)

#-----------------
# MEX Compiler
#-----------------

MEX_Compiler_BASIC =  @$(COMPILER_COMMAND_FILE)

MEX_CFLAGS += $(MEX_Compiler_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################


MINGW_C_STANDARD_OPTS = $(C_STANDARD_OPTS)


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) @$(CMD_FILE)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.obj : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : C:/Users/segigar/ONEDRI~1/Desktop/ABB/test/GENCOD~1/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : C:/Users/segigar/ONEDRI~1/Desktop/ABB/test/GENCOD~1/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


executePlan_data.obj : $(START_DIR)/executePlan_data.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


executePlan_initialize.obj : $(START_DIR)/executePlan_initialize.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


executePlan_terminate.obj : $(START_DIR)/executePlan_terminate.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


executePlan.obj : $(START_DIR)/executePlan.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


controller.obj : $(START_DIR)/controller.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


deal.obj : $(START_DIR)/deal.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ForwardKinematics.obj : $(START_DIR)/ForwardKinematics.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


eye.obj : $(START_DIR)/eye.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ListGet.obj : $(START_DIR)/ListGet.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Joint.obj : $(START_DIR)/Joint.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


any1.obj : $(START_DIR)/any1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


allOrAny.obj : $(START_DIR)/allOrAny.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


applyVectorFunction.obj : $(START_DIR)/applyVectorFunction.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


all.obj : $(START_DIR)/all.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Skew.obj : $(START_DIR)/Skew.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mtimes.obj : $(START_DIR)/mtimes.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mtimes1.obj : $(START_DIR)/mtimes1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


abs.obj : $(START_DIR)/abs.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


abs1.obj : $(START_DIR)/abs1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sqrt.obj : $(START_DIR)/sqrt.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sqrt1.obj : $(START_DIR)/sqrt1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Quaternion.obj : $(START_DIR)/Quaternion.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


DerProd.obj : $(START_DIR)/DerProd.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Exponential.obj : $(START_DIR)/Exponential.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sin.obj : $(START_DIR)/sin.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sin1.obj : $(START_DIR)/sin1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


cos.obj : $(START_DIR)/cos.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


cos1.obj : $(START_DIR)/cos1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


InvAdjoint.obj : $(START_DIR)/InvAdjoint.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


DerSum.obj : $(START_DIR)/DerSum.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ListSet.obj : $(START_DIR)/ListSet.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


DynamicMatrices.obj : $(START_DIR)/DynamicMatrices.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(COMPILER_COMMAND_FILE) $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(subst /,\,$(PRODUCT))
	$(RM) $(subst /,\,$(ALL_OBJS))
	$(ECHO) "### Deleted all derived files."


