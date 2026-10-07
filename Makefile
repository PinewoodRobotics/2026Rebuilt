TEAM_NUMBER=4765

build:
	export JAVA_HOME=$(/usr/libexec/java_home -v 17)
	./gradlew build

deploy:
	export JAVA_HOME=$(/usr/libexec/java_home -v 17)
	./gradlew deploy -PteamNumber=$(TEAM_NUMBER)