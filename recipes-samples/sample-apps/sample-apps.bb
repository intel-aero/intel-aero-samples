DESCRIPTION = "Sample apps and libraries for tasks like spidev device access etc."
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0;md5=801f80980d171dd6425610833a22dbe6"

SRC_URI = "git://github.com/intel-aero/sample-apps.git;protocol=https"
SRCREV = "b655ddce86e11d21af4bc556e23854e1eb4dfc8b"
S = "${WORKDIR}/git"

SRC_FILE = "${WORKDIR}/git/spidev-app/spi_xfer.c"

INHIBIT_PACKAGE_STRIP = "1"

do_compile() {
    ${CC} spidev-app/spi_xfer.c -o spi_xfer
}

do_install() {
    install -d ${D}${bindir}/
    install spi_xfer ${D}${bindir}/
    install -d ${D}${datadir}/sample_files
    install -m 0444 ${SRC_FILE} ${D}${datadir}/sample_files/
}
FILES_${PN} += "${datadir}/sample_files"
