################################################################################
#
# zlog
#
################################################################################

ZLOG_VERSION = 1.2.15
ZLOG_SITE = $(call github,HardySimpson,zlog,$(ZLOG_VERSION))
ZLOG_LICENSE = LGPL-2.1
ZLOG_LICENSE_FILES = COPYING
ZLOG_CPE_ID_VENDOR = zlog_project
ZLOG_INSTALL_STAGING = YES

# 0001-Fix-stack-buffer-overflow-at-zlog_conf_build_with_file.patch
ZLOG_IGNORE_CVES += CVE-2021-43521

define ZLOG_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE1) CC="$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_LDFLAGS)" \
		-C $(@D) all
endef

define ZLOG_INSTALL_STAGING_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) PREFIX=$(STAGING_DIR)/usr -C $(@D) install
endef

define ZLOG_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) PREFIX=$(TARGET_DIR)/usr -C $(@D) install
endef

# Climate Drive2 dependency manually added to pickup custom kernel headers
ZLOG_DEPENDENCIES += kernel_headers

$(eval $(generic-package))
