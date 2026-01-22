N_PROG=stereo_app

all: ${N_PROG}

${N_PROG}:
	${CXX} -std=c++14 ${N_PROG}.cpp capture.cpp image.cpp display.cpp mode.cpp \
	-I${SDKTARGETSYSROOT}/usr/include/opencv4 \
	-L${SDKTARGETSYSROOT}/usr/lib \
	-lopencv_core \
	-lopencv_imgcodecs \
	-lopencv_imgproc \
	-lopencv_highgui \
	-lopencv_video \
	-lopencv_videoio \
	-lopencv_features2d \
	-lopencv_calib3d \
	-lpthread -O2 -ldl ${LDFLAGS} -o ${N_PROG}

clean:
	rm -rf *.o
	rm -rf ${N_PROG}
