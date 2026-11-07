TEAM_NUMBER=4765
WPILIB_JDK := $(HOME)/wpilib/2027_alpha7/jdk
JAVA_HOME := $(if $(wildcard $(WPILIB_JDK)/bin/java),$(WPILIB_JDK),$(shell /usr/libexec/java_home -v 25 2>/dev/null))
export JAVA_HOME

.PHONY: build deploy

build:
	./gradlew build

deploy:
	./gradlew deploy -PteamNumber=$(TEAM_NUMBER)
