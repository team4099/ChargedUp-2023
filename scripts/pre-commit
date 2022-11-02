#!/bin/sh

echo "*****Running spotless check******"

git stash -q --keep-index

./gradlew spotlessCheck

status=$?

git stash pop -q

echo "*****Done with spotless check******"

exit $status
