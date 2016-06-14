# Variables
BOARD_DIR = $(BASE_DIR)/board/drivers
OBJ_DIRS += $(APP_DIR)/$(BUILD)/$(SOC)/$(TARGET)
DRIVER_SOURCES = $(foreach drv, $(PERIPH), $(wildcard $(BOARD_DIR)/$(drv)/*.c))
OBJECTS += $(addprefix $(APP_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)/,$(notdir $(DRIVER_SOURCES:.c=.o)))
GENERATED_DIRS += $(BOARD_DIR)/$(BUILD)

CFLAGS += -I$(BOARD_DIR)

# Build C files
$(APP_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)/%.o: $(BOARD_DIR)/bmx1xx/%.c
	$(call mkdir, $(APP_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ))
	$(CC) $(CFLAGS) -c -o $@ $<

# Build C files
$(APP_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)/%.o: $(BOARD_DIR)/rf24/%.c
	$(call mkdir, $(APP_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ))
	$(CC) $(CFLAGS) -c -o $@ $<
