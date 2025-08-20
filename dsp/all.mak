include $(HEXAGON_SDK_ROOT)/build/make.d/hexagon_vs.min

include hexagon_deps.min

SUPPORTED_VS_CLEAN = $(foreach d,$(SUPPORTED_VS),$(d)_CLEAN)

.PHONEY: tree tree_clean $(SUPPORTED_VS) $(SUPPORTED_VS_CLEAN)

tree: $(SUPPORTED_VS)

tree_clean: $(SUPPORTED_VS_CLEAN)

$(SUPPORTED_VS):
	$(MAKE) V=$(@) tree

$(SUPPORTED_VS_CLEAN):
	$(MAKE) V=$(patsubst %_CLEAN,%,$(@)) tree_clean
