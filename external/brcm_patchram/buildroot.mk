BRCM_PATCHRAM_VERSION = 1.1
BRCM_PATCHRAM_SITE = $(BR2_EXTERNAL_DTO_PATH)/external/brcm_patchram
BRCM_PATCHRAM_SITE_METHOD = local

define BRCM_PATCHRAM_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) $(TARGET_CONFIGURE_OPTS) -C $(@D) brcm_patchram_plus
endef

define BRCM_PATCHRAM_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/brcm_patchram_plus $(TARGET_DIR)/usr/bin
endef

$(eval $(generic-package))
