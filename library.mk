
MFRC522_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
CSRC += $(wildcard $(MFRC522_DIR)/mfrc522.c)
EXTRAINCDIRS += $(MFRC522_DIR)

CFLAGS += -DHAS_MFRC522
