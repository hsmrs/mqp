***********************************************************************
***	HSMRS MQP - Donald Bourque - Thomas DeSilva - Nicholas Otero	***
***																	***
***					HSMRS Graphical User Interface					***
***********************************************************************

This is the top directory of the hsmrs_gui project. All of the substantive
code which is used for the GUI can be found in the 
project/src/main/java/com/github/hsmrs_gui/project directory. In order to 
build the java code, you must be in the top directory in the command line
and run the following command:

./gradlew installApp

This will build the java code and create an executable in the 
project/build/install/project/bin/project directory called
com.github.hsmrs_gui.project.GuiNode.  If running the GUI, there is a
script called run_hsmrs_gui.bash that can be run from the top directory
to run the GUI.  

To add dependencies to the project, add a compile command along with a
maven repository to the settings.gradle file. An example is included within
the file.
