name=BayesBot
rtpath= #path to rt.jar
robocodepath= # path to robocode
cp=$rtpath:$robocodepath/libs/robocode.jar:

find . -type f -name '*.class' -delete\
\
&&\
\
javac -deprecation -g -source 1.6 -encoding UTF-8 \
    -classpath $cp \
    bayes/$name.java \
\
&& \
\
mkdir -p $robocodepath/robots/bayes \
\
&& \
\
cp -r ./bayes/* $robocodepath/robots/bayes
