LCM_JAR = /usr/local/share/java/lcm.jar

LCMTYPES_JAR = lcmtypes.jar
BUILD_PATH = build
SOURCE_PATH = src

# Construct the pathnames to the C types
LCMTYPES_NAMES := $(basename $(shell cd ../lcmtypes && ls *.lcm))
LCMTYPES_JAVA := $(addsuffix .java, $(addprefix $(SOURCE_PATH)/lcmtypes/, $(LCMTYPES_NAMES)))
LCMTYPES_CLASS := $(addsuffix .class, $(addprefix $(BUILD_PATH)/lcmtypes/, $(LCMTYPES_NAMES)))

all: $(LCMTYPES_JAR)

src/lcmtypes/%.java: ../lcmtypes/%.lcm
	@echo "\t$<"
	@lcm-gen -j --jpath=src --jdefaultpkg=lcmtypes $<

.LCMTYPES_CLASS: $(BUILD_PATH) $(LCMTYPES_JAVA)
	@echo "[java/lcmtypes]"
	@echo "\t$(LCMTYPES_JAVA)"
	@javac -classpath $(LCM_JAR) -d $(BUILD_PATH) $(LCMTYPES_JAVA)
	@touch .LCMTYPES_CLASS

$(LCMTYPES_JAR): $(BUILD_PATH) .LCMTYPES_CLASS $(PLUGINS_CLASS)
	@echo "[java/lcmtypes.jar]"
	@echo "$(LCMTYPES_JAR)"
	@jar cf $(LCMTYPES_JAR) -C $(BUILD_PATH) .

$(BUILD_PATH):
	@mkdir -p $(BUILD_PATH)

clean:
	@echo "clean [java/lcmtypes]"; rm -f $(SOURCE_PATH)/lcmtypes/*
	@echo "clean [java/build]"; rm -rf build/*
	@echo "clean [java/lcmtypes.jar]"; rm -f $(LCMTYPES_JAR)
	@rm -f *~ .LCMTYPES_CLASS
