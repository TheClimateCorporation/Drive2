################################################################################
#
# iperf3
#
################################################################################

IPERF3_VERSION = 3.10.1
IPERF3_SITE = https://downloads.es.net/pub/iperf
IPERF3_SOURCE = iperf-$(IPERF3_VERSION).tar.gz
IPERF3_LICENSE = BSD-3-Clause, BSD-2-Clause, MIT
IPERF3_LICENSE_FILES = LICENSE
IPERF3_CPE_ID_VENDOR = es

IPERF3_CONF_ENV += CFLAGS="$(TARGET_CFLAGS) -D_GNU_SOURCE"

ifeq ($(BR2_PACKAGE_OPENSSL),y)
# We intentionally don't pass --with-openssl, otherwise pkg-config is
# not used, and indirect libraries are not picked up when static
# linking.
IPERF3_DEPENDENCIES += host-pkgconf openssl
else
IPERF3_CONF_OPTS += --without-openssl
endif

ifeq ($(BR2_PACKAGE_SYSTEMD),y)
define IPERF3_INSTALL_SERVICE
	$(INSTALL) -D -m 0644 $(@D)/contrib/iperf3.service $(TARGET_DIR)/usr/lib/systemd/system/iperf3.service
	mkdir -p $(TARGET_DIR)/etc/systemd/system/multi-user.target.wants
        ln -sf ../../../../usr/lib/systemd/system/iperf3.service \
                $(TARGET_DIR)/etc/systemd/system/multi-user.target.wants/iperf3.service

endef
IPERF3_POST_INSTALL_TARGET_HOOKS += IPERF3_INSTALL_SERVICE
endif

$(eval $(autotools-package))
