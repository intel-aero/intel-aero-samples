DESCRIPTION = "Sample apps with Aero"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0;md5=801f80980d171dd6425610833a22dbe6"

SRC_URI = "git://github.com/intel-aero/sample-apps.git;protocol=https"
SRCREV = "c47cbce1588f2e73f02b6c96ab70f6783900aaa9"
S = "${WORKDIR}/git"
V = "${S}/capturev4l2"
LIBS = "-lstdc++ -lstlport"
CFLAGS = "-g -std=c++11 -Wall -DSTDC99 -D_REENTRANT -DATOMISP_CSS2 -DATOMISP_CSS2 -DATOMISP_CSS21"

SRC_FILE = "${S}/spidev-app/spi_xfer.c"

INHIBIT_PACKAGE_STRIP = "1"

do_compile() {
    ${CC} spidev-app/spi_xfer.c -o spi_xfer
    ${CXX} -I${V} -c -o ${V}/atomisp_obj.o ${V}/atomisp_obj.cpp ${CFLAGS} ${LIBS}
    ${CXX} -I${V} -c -o ${V}/v4l2_obj.o ${V}/v4l2_obj.cpp ${CFLAGS} ${LIBS}
    ${CXX} -I${V} -c -o ${V}/capture.o  ${V}/capture.cpp ${CFLAGS} ${LIBS}
    ${CXX} -pthread -o ${V}/capture_example ${V}/capture.o ${V}/v4l2_obj.o ${V}/atomisp_obj.o -I${V}
}

do_install() {
    install -d ${D}${bindir}/
    install spi_xfer ${D}${bindir}/
    install -d ${D}${datadir}/sample_files
    install -m 0444 ${SRC_FILE} ${D}${datadir}/sample_files/
}
FILES_${PN} += "${datadir}/sample_files"
