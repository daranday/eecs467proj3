include ../common.mk

CFLAGS  =  $(CFLAGS_STD) $(CFLAGS_VX) $(CFLAGS_VX_GTK) $(CFLAGS_GTK) $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O4
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_VX_GTK) $(LDFLAGS_GTK) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES) $(LDFLAGS_CV)
LIBDEPS = $(call libdeps, $(LDFLAGS))
CXXFLAGS = $(CXXFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
         $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_MATH) \
	 $(CFLAGS_COMMON) \
	 $(CFLAGS_LCMTYPES) \
	 # -O2

LIB_EECS467 = $(LIB_PATH)/libeecs467.a
LIBEECS467_OBJS = ../apps/eecs467_util.o

BIN_A3_MASK = $(BIN_PATH)/a3_mask
BIN_A3_COLOR_PICKER = $(BIN_PATH)/a3_color_picker
BIN_A3_BLOB_DETECTOR = $(BIN_PATH)/a3_blob_detector
BIN_A3_INVERSE_KINEMATICS = $(BIN_PATH)/a3_inverse_kinematics
BIN_A3_IMAGE_TO_ARM_COORD = $(BIN_PATH)/a3_image_to_arm_coord
BIN_A3_AI = $(BIN_PATH)/a3_ai
BIN_A3_PICASSO = $(BIN_PATH)/a3_picasso
BIN_A3_ARM_TEST = $(BIN_PATH)/a3_arm_test
BIN_OPENCV = $(BIN_PATH)/opencv

ALL = $(BIN_A3_MASK) $(BIN_A3_COLOR_PICKER) \
		$(BIN_A3_PICASSO) $(BIN_A3_ARM_TEST) # $(BIN_OPENCV)# $(BIN_OPENCV) $(BIN_A3_BLOB_DETECTOR) $(BIN_A3_INVERSE_KINEMATICS) $(BIN_A3_IMAGE_TO_ARM_COORD)   $(BIN_A3_AI)\

all: $(ALL)


$(BIN_A3_MASK): a3_mask.o $(LIBDEPS) $(LIBEECS467_OBJS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

$(BIN_A3_COLOR_PICKER): a3_color_picker.o $(LIBDEPS) $(LIBEECS467_OBJS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

# $(BIN_A3_BLOB_DETECTOR): a3_blob_detector.o $(LIBDEPS) $(LIBEECS467_OBJS)
# 	@echo "t\$@"
# 	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

# $(BIN_A3_INVERSE_KINEMATICS): a3_inverse_kinematics.o $(LIBDEPS) $(LIBEECS467_OBJS)
# 	@echo "t\$@"
# 	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

# $(BIN_A3_IMAGE_TO_ARM_COORD): a3_image_to_arm_coord.o $(LIBDEPS)
# 	@echo "t\$@"
# 	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

# $(BIN_A3_AI): a3_ai.o $(LIBDEPS)
# 	@echo "t\$@"
# 	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

$(BIN_A3_PICASSO): a3_picasso.o a3_blob_detector.o a3_inverse_kinematics.o a3_image_to_arm_coord.o a3_ai.o $(LIBDEPS) $(LIBEECS467_OBJS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

$(BIN_A3_ARM_TEST): a3_arm_test.o $(LIBDEPS) $(LIBEECS467_OBJS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

#$(BIN_OPENCV):  opencv.o  a3_blob_detector.o a3_inverse_kinematics.o a3_image_to_arm_coord.o a3_ai.o $(LIBDEPS) $(LIBEECS467_OBJS)
#	@echo "\t$@"
#	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)


clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
